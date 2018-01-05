//
// Copyright (c) 2017 CNRS
// Authors: Mathieu Geisert
//          (adapted from Pierre Fernbach's code: dynamic-validation.cc)
//
// This file is part of hpp-rbprm
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

# include <hpp/rbprm/learning/learning-validation.hh>
# include <hpp/util/debug.hh>
# include <hpp/rbprm/planner/rbprm-node.hh>
# include <hpp/model/configuration.hh>
#include <hpp/util/timer.hh>
#include "utils/algorithms.h"
namespace hpp {
namespace rbprm {

struct SurfaceData{

    SurfaceData():normal_(),centroid_(),collisionObject_(),intersection_()
    {}

    SurfaceData(geom::Point normal, geom::Point centroid, model::CollisionObjectPtr_t collisionObject, geom::T_Point intersection)
        : normal_(normal),centroid_(centroid),collisionObject_(collisionObject),intersection_(intersection)
    {}

    geom::Point normal_;
    geom::Point centroid_;
    model::CollisionObjectPtr_t collisionObject_;
    geom::T_Point intersection_;

};

typedef std::list<SurfaceData> SurfaceDatas_t;

LearningValidationPtr_t LearningValidation::create(GMMPtr_t gmm,
const model::RbPrmDevicePtr_t& robot,
const std::vector<std::string>& filter,
const std::map<std::string, std::vector<std::string> >& affFilters,
const std::map<std::string, std::vector<model::CollisionObjectPtr_t> >& affordances,
const core::ObjectVector_t& geometries)
{
    LearningValidation* ptr = new LearningValidation (gmm,robot, filter, affFilters,
                                                      affordances, geometries);
    return LearningValidationPtr_t (ptr);
}


LearningValidation::LearningValidation (GMMPtr_t gmm,
                                        const model::RbPrmDevicePtr_t& robot,
                                        const std::vector<std::string>& filter,
                                        const std::map<std::string,std::vector<std::string> >& affFilters,
                                        const std::map<std::string,
                                        std::vector<model::CollisionObjectPtr_t> >& affordances,
                                        const core::ObjectVector_t& geometries)
    :parent_t(robot, filter, affFilters,affordances, geometries)
    ,gmm_(gmm)
{

}

geom::T_Point displayCollisionObjectVertices(const std::string& nameObj,geom::BVHModelOBConst_Ptr_t model){
    geom::T_Point vertices;
    std::ostringstream ss;
    ss<<"[";
    for(int i = 0 ; i < model->num_vertices ; ++i)
    {
      vertices.push_back(Eigen::Vector3d(model->vertices[i][0], model->vertices[i][1], model->vertices[i][2]));
      ss<<"["<<model->vertices[i][0]<<","<<model->vertices[i][1]<<","<<model->vertices[i][2]<<"]";
      if(i< (model->num_vertices-1))
        ss<<",";
    }
    ss<<"]";
    hppDout(info,"vertices of object : "<<nameObj<< " ( "<<model->num_vertices<<" ) ");
    hppDout(notice,""<<ss.str());
    return vertices;
}

std::string displayPoints(geom::T_Point points){
    std::ostringstream ss;
    ss<<"[";

    for(geom::CIT_Point ip = points.begin() ; ip != points.end() ; ++ip){
        ss<<"["<<(*ip)[0]<<","<<(*ip)[1]<<","<<(*ip)[2]<<"],";
    }

    ss<<"]";
    return ss.str();
}

SurfaceDatas_t computeSurfaceData(core::AllCollisionsValidationReportPtr_t romReport){
    SurfaceDatas_t surfaces;
    hppDout(notice,"There is "<<romReport->collisionReports.size()<<" object in collision with the ROM");
    std::ostringstream ssCenters;
    ssCenters<<"[";
    for(std::vector<core::CollisionValidationReportPtr_t>::const_iterator itCollision = romReport->collisionReports.begin() ; itCollision != romReport->collisionReports.end() ; ++itCollision){ // for all avoidance object in collision with the ROM
        core::CollisionObjectPtr_t obj1 = (*itCollision)->object1; // should be the rom
        core::CollisionObjectPtr_t obj2 = (*itCollision)->object2; // should be the environnement
        hppDout(notice,"collision between : "<<obj1->name() << " and "<<obj2->name());
        fcl::CollisionResult result = (*itCollision)->result;


        // ##  get intersection between the two objects :
        geom::BVHModelOBConst_Ptr_t model1 =  geom::GetModel(obj1->fcl());
        geom::BVHModelOBConst_Ptr_t model2 =  geom::GetModel(obj2->fcl());
        displayCollisionObjectVertices(obj1->name(),model1); // DEBUG only
        displayCollisionObjectVertices(obj2->name(),model2);

        geom::Point pn; // plan normal
        geom::T_Point plane = geom::intersectPolygonePlane(model1,model2,pn);

        geom::T_Point hull; // convex hull of the plane
        if(plane.size() > 0){
          hull = geom::compute3DIntersection(plane,geom::convertBVH(model2));
        }else{
            hppDout(notice,"Error when computing plane.");
        }

        if(hull.size() > 0){
            // compute center point of the hull
            geom::Point center = geom::center(hull.begin(),hull.end());
            SurfaceData data(pn,center,obj2,hull); // How can we be sure that obj2 is the environnement ?
            surfaces.push_back(data);
            hppDout(notice,"Add a Surface Data : ");
            hppDout(notice,"Normal = "<<pn.transpose());
            hppDout(notice,"center = ["<<center[0]<<","<<center[1]<<","<<center[2]<<"]");
            ssCenters<<" ["<<center[0]<<","<<center[1]<<","<<center[2]<<"],";
            hppDout(notice,"obj name : "<<obj2->name());
            hppDout(notice,"intersection : "<<displayPoints(hull));
        }else{
            hppDout(notice,"Error when computing intersection.");
        }



    } // end for all collisions objects

    hppDout(notice,"List of all centers for this limb : "<<ssCenters.str()<<"]");
    return surfaces;
}

SurfaceDatas_t computeSurfaceDataForLimb(const std::string& limbName,core::RbprmValidationReportPtr_t rbReport){

    if (rbReport->ROMFilters.find(limbName) == rbReport->ROMFilters.end()){
        hppDout(notice,"Error : ROM report does not contain entry for limb : "<<limbName);
        return SurfaceDatas_t();
    }
    if (rbReport->ROMFilters.at(limbName)){ // the ROM is in collision
        core::AllCollisionsValidationReportPtr_t allCollisionReport = boost::dynamic_pointer_cast<core::AllCollisionsValidationReport>(rbReport->ROMReports.at(limbName));
        if(allCollisionReport){
            hppDout(notice,"Compute surface data for rom : "<<limbName);
            return computeSurfaceData(allCollisionReport);
        }else{
            hppDout(notice,"Error : ROM report cannot be cast correctly. Have you correctly set computeALlCollisions(true) ?");
            return SurfaceDatas_t();
        }
    }else{ // the ROM is not in collision
        return SurfaceDatas_t();
    }
}


bool LearningValidation::validateRoms(const core::Configuration_t& config,
                  const std::vector<std::string>& filter,
                   core::ValidationReportPtr_t &validationReport){
    computeAllContacts(true);
    parent_t::validateRoms(config,filter,validationReport);
    core::RbprmValidationReportPtr_t rbReport = boost::dynamic_pointer_cast<core::RbprmValidationReport> (validationReport);
    hppDout(notice,"[LEARNING] : begin validateRoms from learning-validation");
    hppDout(notice,"For config "<<model::displayConfig(config));
    // test validity of the given report :
    if(!rbReport)
    {
      hppDout(error,"Learning validation : Validation Report cannot be cast");
      return false;
    }
    if(rbReport->trunkInCollision)
    {
      hppDout(error,"Learning validation : trunk is in collision");
      return false;
    }
    if(!rbReport->romsValid)
    {
      hppDout(error,"Learning validation : roms filter not respected");
      return false;
    }

    SurfaceDatas_t surfaces;
    for(T_RomValidation::const_iterator itVal = romValidations_.begin() ; itVal != romValidations_.end() ; ++itVal){ // iterate over all limbs
        const std::string& itLimb(itVal->first);
        surfaces = computeSurfaceDataForLimb(itLimb,rbReport);
        if(surfaces.empty()){ // no collision or error (see logs)
            // TODO ??
            hppDout(notice,"No collisions or errors occured for limb : "<<itLimb);
            return false;
        }
        hppDout(notice,"SurfacesData computed for limb : "<<itLimb<<"; number of surfaces : "<<surfaces.size());


        // TODO : use 'surfaces' directly in the loop or store them in a map (keys = limbName) ?

    } // end for all limbs



    return rbReport->romsValid; // TODO : remove and replace with test with learning
}




}//rbprm
}//hpp
