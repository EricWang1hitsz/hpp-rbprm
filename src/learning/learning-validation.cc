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

namespace hpp {
namespace rbprm {

LearningValidationPtr_t LearningValidation::create(GMM gmm,
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


LearningValidation::LearningValidation (GMM gmm,
                                        const model::RbPrmDevicePtr_t& robot,
                                        const std::vector<std::string>& filter,
                                        const std::map<std::string,
                                        std::vector<std::string> >& affFilters,
                                        const std::map<std::string,
                                        std::vector<model::CollisionObjectPtr_t> >& affordances,
                                        const core::ObjectVector_t& geometries)
    :parent_t(robot, filter, affFilters,affordances, geometries)
    ,gmm_(gmm)
{

}




}//rbprm
}//hpp
