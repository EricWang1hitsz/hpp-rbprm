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

struct surfaceData{

    surfaceData():normal_(),centroid_(),collisionObject_()
    {}

    surfaceData(geom::Point normal, geom::Point centroid, model::CollisionObjectPtr_t collisionObject)
        : normal_(normal),centroid_(centroid),collisionObject_(collisionObject)
    {}

    geom::Point normal_;
    geom::Point centroid_;
    model::CollisionObjectPtr_t collisionObject_;

};

typedef std::list<surfaceData> surfaceDatas_t;

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

surfaceDatas_t computeSurfaceDataForLimb(const std::string& limbName,core::CollisionValidationReportPtr_t romReport){
    surfaceDatas_t surfaces;


    return surfaces;
}


bool LearningValidation::validateRoms(const core::Configuration_t& config,
                  const std::vector<std::string>& filter,
                   core::ValidationReportPtr_t &validationReport){
    computeAllContacts(true);
    parent_t::validateRoms(config,filter,validationReport);
    core::RbprmValidationReportPtr_t rbReport = boost::dynamic_pointer_cast<core::RbprmValidationReport> (validationReport);

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

    surfaceDatas_t surfaces; // use it directly in the loop or store them in a map (keys = limbName) ?
    for(T_RomValidation::const_iterator itVal = romValidations_.begin() ; itVal != romValidations_.end() ; ++itVal){ // iterate over all limbs
        const std::string& itLimb(itVal->first);
        if (rbReport->ROMFilters.find(itLimb) == rbReport->ROMFilters.end()){
            hppDout(notice,"Error : ROM report does not contain entry for limb : "<<itLimb);
            return false;
        }
        if (rbReport->ROMFilters.at(itLimb)){
            surfaces = computeSurfaceDataForLimb(itLimb,rbReport->ROMReports.at(itLimb));







        }else{ // rom for limb 'itLimb' is not in collision.
            // TODO ???
        }
    } // end for all limbs


}




}//rbprm
}//hpp
