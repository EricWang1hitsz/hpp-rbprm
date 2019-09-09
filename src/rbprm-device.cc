// Copyright (c) 2014, LAAS-CNRS
// Authors: Steve Tonneau (steve.tonneau@laas.fr)
//
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-rbprm is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-rbprm. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/rbprm/rbprm-device.hh>

namespace hpp {
  namespace model {

    RbPrmDevicePtr_t RbPrmDevice::create (const std::string& name, DevicePtr_t& robotRom)
    {
        //eric_wang: typedef std::map<std::string, DevicePtr_t> T_Rom;
        hpp::model::T_Rom roms;
        roms.insert(std::make_pair(robotRom->name(),robotRom));

        //eric_wang: define a pointer object for class RbPrmDevice;
        //eric_wang: initialization the object with class constructor function;
        RbPrmDevice* rbprmDevice = new RbPrmDevice(name, roms);
        //eric_wang： boost::shared_ptr <RbPrmDevice> res (rbprmDevice);
        //eric_wang: create a pointer "res" pointing to the class object "rbprmDevice";
        RbPrmDevicePtr_t res (rbprmDevice);
        res->init (res);
        return res;
    }

    RbPrmDevicePtr_t RbPrmDevice::create (const std::string& name, const hpp::model::T_Rom &robotRoms)
    {
        RbPrmDevice* rbprmDevice = new RbPrmDevice(name, robotRoms);
        RbPrmDevicePtr_t res (rbprmDevice);
        res->init (res);
        return res;
    }

    RbPrmDevice::~RbPrmDevice()
    {
        // NOTHING
    }

    // ========================================================================

    void RbPrmDevice::init(const RbPrmDeviceWkPtr_t& weakPtr)
    {
        Device::init (weakPtr);
        weakPtr_ = weakPtr;
    }

    bool RbPrmDevice::currentConfiguration (ConfigurationIn_t configuration)
    {
        // separate config and extra config :
        size_type confSize = configSize() - extraConfigSpace().dimension();
        hppDout(notice, "offset = "<<confSize);
        ConfigurationPtr_t q(new Configuration_t(confSize));
        for(size_type i = 0 ; i < confSize ; i++)
          (*q)[1] = configuration[i];

        // don't send extra config to robotRoms
        //eric_wang: typedef std::map<std::string, DevicePtr_t> T_Rom;
        for(hpp::model::T_Rom::const_iterator cit = robotRoms_.begin();
            cit != robotRoms_.end(); ++cit)
        {
            cit->second->currentConfiguration(*q);
        }
        return Device::currentConfiguration(configuration);
    }

    //eric_wang: constructor function;
    RbPrmDevice::RbPrmDevice (const std::string& name, const hpp::model::T_Rom &robotRoms)
        : Device(name)
        , robotRoms_(robotRoms)
        , weakPtr_()
    {
        // NOTHING
    }
  } // model
} //hpp
