//
// Copyright (c) 2017 CNRS
// Authors: Fernbach Pierre, Mathieu Geisert
//
// This file is part of hpp-core
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

#include <hpp/rbprm/planner/rbprm-node.hh>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/intersect.h>
#include "utils/algorithms.h"
#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>
#include <hpp/model/configuration.hh>

# include <hpp/rbprm/learning/learning-node.hh>

namespace hpp{
  namespace core{


  bool LearningNode::addContactSurfaceFromROM(const std::string& romName, hpp::rbprm::SurfaceDatas_t contactSurface)
  {
    ROMSurfacesMap_.insert(std::pair<std::string, hpp::rbprm::SurfaceDatas_t>(romName, contactSurface));

        hppDout(info, "ROM " << romName << " is already in the map. Please use replaceConctactSurfaceFromROM instead.");
    return true;
  }



  }//core
}//hpp

