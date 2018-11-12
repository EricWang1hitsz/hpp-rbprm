
// Copyright (C) 2018 LAAS-CNRS
// Author: Pierre Fernbach
//
// This file is part of the hpp-rbprm.
//
// hpp-rbprm is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// test-hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-core.  If not, see <http://www.gnu.org/licenses/>.



#ifndef TOOLSFULLBODY_HH
#define TOOLSFULLBODY_HH

#include <hpp/rbprm/rbprm-fullbody.hh>
#include <hpp/rbprm/rbprm-state.hh>
#include <pinocchio/parsers/urdf.hpp>
#include <hpp/pinocchio/urdf/util.hh>

using namespace hpp;
using namespace rbprm;

/*

#~ AFTER loading obstacles

rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.105]
rLegLimbOffset=[0,0,-0.035]#0.035
rLegNormal = [0,0,1]
rLegx = 0.09; rLegy = 0.05
#fullBody.addLimbDatabase("./db/hrp2_rleg_db.db",rLegId,"forward")
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 100000, "fixedStep1", 0.01,"_6_DOF",limbOffset=rLegLimbOffset)
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/hrp2_rleg_db.db")

lLeg = 'LLEG_JOINT0'
lLegOffset = [0,0,-0.105]
lLegLimbOffset=[0,0,0.035]
lLegNormal = [0,0,1]
lLegx = 0.09; lLegy = 0.05
#fullBody.addLimbDatabase("./db/hrp2_lleg_db.db",lLegId,"forward")
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 100000, "fixedStep1", 0.01,"_6_DOF",limbOffset=lLegLimbOffset)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(lLegId, "./db/hrp2_lleg_db.db")
fullBody.setReferenceConfig (q_ref)

*/


RbPrmFullBodyPtr_t loadHRP2(){
    const std::string robotName("hrp2_14");
    const std::string rootJointType ("freeflyer");
    const std::string packageName ("hrp2_14_description");
    const std::string modelName ("hrp2_14");
    const std::string urdfSuffix ("_reduced");
    const std::string srdfSuffix ("");

    hpp::pinocchio::DevicePtr_t device = hpp::pinocchio::Device::create (robotName);
    //hpp::pinocchio:: (device,rootJointType, packageName, modelName, urdfSuffix,srdfSuffix);
    device->setDimensionExtraConfigSpace(6);

    RbPrmFullBodyPtr_t fullBody = RbPrmFullBody::create(device);


    core::Configuration_t q_ref(device->configSize());
    q_ref<<0.1, -0.82, 0.648702, 1.0, 0.0 , 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0,0,0,0,0,0;
    fullBody->referenceConfig(q_ref);

     // add limbs :
    const std::string rLegId("hrp2_rleg_rom");
    const std::string rLeg("RLEG_JOINT0");
    const std::string rfeet("RLEG_JOINT5");
    fcl::Vec3f rLegOffset( 0,0,-0.105);
    fcl::Vec3f rLegLimbOffset(0,0,-0.035);
    fcl::Vec3f rLegNormal(0,0,1);
    double legX = 0.09;
    double legY = 0.05;
    fullBody->AddLimb(rLegId,rLeg,rfeet,rLegOffset,rLegLimbOffset,rLegNormal,legX,legY,hpp::core::ObjectStdVector_t(),1000,"fixedStep1",0.01,hpp::rbprm::_6_DOF,false,false,std::string(),0.3);

    const std::string lLegId("hrp2_lleg_rom");
    const std::string lLeg("LLEG_JOINT0");
    const std::string lfeet("LLEG_JOINT5");
    fcl::Vec3f lLegOffset(0,0,-0.105);
    fcl::Vec3f lLegLimbOffset(0,0,0.035);
    fcl::Vec3f lLegNormal(0,0,1);
    fullBody->AddLimb(lLegId,lLeg,lfeet,lLegOffset,lLegLimbOffset,lLegNormal,legX,legY,hpp::core::ObjectStdVector_t(),1000,"fixedStep1",0.01,hpp::rbprm::_6_DOF,false,false,std::string(),0.3);

    return fullBody;

}

RbPrmFullBodyPtr_t loadHyQ(){
    const std::string robotName("hyq");
    const std::string rootJointType ("freeflyer");
    const std::string packageName ("hyq_description");
    const std::string modelName ("hyq");
    const std::string urdfSuffix ("");
    const std::string srdfSuffix ("");

    hpp::pinocchio::DevicePtr_t device = hpp::pinocchio::Device::create (robotName);
    //hpp::pinocchio:: (device,rootJointType, packageName, modelName, urdfSuffix,srdfSuffix);


    hpp::pinocchio::urdf::loadRobotModel(device, rootJointType, packageName, modelName,
    urdfSuffix, srdfSuffix);
    // [-2,5, -1, 1, 0.3, 4]

  /*DevicePtr_t robot =
    hpp::pinocchio::humanoidSimple("test", true,
        (Device::Computation_t) (Device::JOINT_POSITION | Device::JACOBIAN));*/
  device->rootJoint()->lowerBound(0, -2);
  device->rootJoint()->lowerBound(1,  5);
  device->rootJoint()->lowerBound(2, -1);
  device->rootJoint()->upperBound(0,  1);
  device->rootJoint()->upperBound(1,  0.3);
  device->rootJoint()->upperBound(2,  4);

    device->setDimensionExtraConfigSpace(6);

    RbPrmFullBodyPtr_t fullBody = RbPrmFullBody::create(device);


    core::Configuration_t q_ref(device->configSize());
    q_ref<<-2.0,
            0.0,
            0.6838277139631803,
            0.0,
            0.0,
            0.0,
            1.0,
            0.14279812395541294,
            0.934392553166556,
            -0.9968239786882757,
            -0.06521258938340457,
            -0.8831796268418511,
            1.150049183494211,
            -0.06927610020154493,
            0.9507443168724581,
            -0.8739975339028809,
            0.03995660287873871,
            -0.9577096766517215,
            0.93846028213260710;
    fullBody->referenceConfig(q_ref);


    const fcl::Vec3f limbOffset(0,0,0);
    fcl::Vec3f legOffset( 0,-0.021,0);
    fcl::Vec3f legNormal(0,1,0);
    double legX = 0.02;
    double legY = 0.02;

     // add limbs :
    const std::string rLegId("rfleg");
    const std::string rLeg("rf_haa_joint");
    const std::string rfeet("rf_foot_joint");
    fullBody->AddLimb(rLegId,rLeg,rfeet,legOffset,limbOffset,legNormal,legX,legY,hpp::core::ObjectStdVector_t(),1000,"random",0.1,hpp::rbprm::_3_DOF,false,false,std::string(),0.3);

    const std::string lLegId("rfleg");
    const std::string lLeg("rf_haa_joint");
    const std::string lfeet("rf_foot_joint");
    fullBody->AddLimb(lLegId,lLeg,lfeet,legOffset,limbOffset,legNormal,legX,legY,hpp::core::ObjectStdVector_t(),1000,"random",0.1,hpp::rbprm::_3_DOF,false,false,std::string(),0.3);

    const std::string rhLegId("rhleg");
    const std::string rhLeg("rh_haa_joint");
    const std::string rhfeet("rh_foot_joint");
    fullBody->AddLimb(rhLegId,rhLeg,rhfeet,legOffset,limbOffset,legNormal,legX,legY,hpp::core::ObjectStdVector_t(),1000,"random",0.1,hpp::rbprm::_3_DOF,false,false,std::string(),0.3);

    const std::string lhLegId("lhleg");
    const std::string lhLeg("lh_haa_joint");
    const std::string lhfeet("lh_foot_joint");
    fullBody->AddLimb(lhLegId,lhLeg,lhfeet,legOffset,limbOffset,legNormal,legX,legY,hpp::core::ObjectStdVector_t(),1000,"random",0.1,hpp::rbprm::_3_DOF,false,false,std::string(),0.3);


    return fullBody;

}

State createState(const RbPrmFullBodyPtr_t& fullBody,core::Configuration_t config,const std::vector<std::string>& limbsInContact){
    fullBody->device_->currentConfiguration(config);
    fullBody->device_->computeForwardKinematics();
    State state;
    state.configuration_ = config;
    for(std::vector<std::string>::const_iterator cit = limbsInContact.begin(); cit != limbsInContact.end(); ++cit)
    {
        rbprm::RbPrmLimbPtr_t limb = fullBody->GetLimbs().at(*cit);
        const std::string& limbName = *cit;
        state.contacts_[limbName] = true;
        const fcl::Vec3f position = limb->effector_.currentTransformation().translation();
        state.contactPositions_[limbName] = position;
        state.contactNormals_[limbName] = limb->effector_.currentTransformation().rotation() * limb->normal_;
        state.contactRotation_[limbName] = limb->effector_.currentTransformation().rotation();
        state.contactOrder_.push(limbName);
    }
    state.nbContacts = state.contactNormals_.size();
    return state;
}

// assume all limbs are in contact
State createState(const RbPrmFullBodyPtr_t& fullBody,core::Configuration_t config){
    std::vector<std::string> limbsInContact;
    for(rbprm::CIT_Limb lit = fullBody->GetLimbs().begin() ; lit != fullBody->GetLimbs().end() ; ++lit){
        limbsInContact.push_back(lit->first);
    }
    return createState(fullBody,config,limbsInContact);
}

#endif // TOOLSFULLBODY_HH
