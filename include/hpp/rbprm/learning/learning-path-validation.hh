//
// Copyright (c) 2017 CNRS
// Authors: Mathieu Geisert
//          (adapted from Pierre Fernbach's code: dynamic-path-validation.hh)
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


#ifndef HPP_RBPRM_LEARNING_PATH_VALIDATION_HH
#define HPP_RBPRM_LEARNING_PATH_VALIDATION_HH
#include <hpp/rbprm/rbprm-path-validation.hh>
#include <hpp/rbprm/learning/learning-validation.hh>


namespace hpp {
  namespace rbprm {


    // forward declaration
    HPP_PREDEF_CLASS (LearningPathValidation);
    // Planner objects are manipulated only via shared pointers
    typedef boost::shared_ptr <LearningPathValidation> LearningPathValidationPtr_t;


    class HPP_RBPRM_DLLAPI LearningPathValidation : public RbPrmPathValidation
    {
    public:
      /// Create an instance and return a shared pointer to the instance
      static LearningPathValidationPtr_t LearningPathValidation::create (const core::DevicePtr_t& robot, const core::value_type& stepSize)
      {
        LearningPathValidation* ptr (new LearningPathValidation(robot, stepSize));
        LearningPathValidationPtr_t shPtr (ptr);
        return shPtr;
      }

      /// validate with custom filter for the rom validation
      virtual bool validate (const core::PathPtr_t& path, bool reverse, core::PathPtr_t& validPart, core::PathValidationReportPtr_t& report,const std::vector<std::string>& filter);

      virtual bool validate (const core::PathPtr_t& path, bool reverse,  core::PathPtr_t& validPart,  core::PathValidationReportPtr_t& report);

      /// add Validator
      void addLearningValidator(const LearningValidationPtr_t& LearningValidation){
        core::DiscretizedPathValidation::add (LearningValidation);
        LearningValidation_=LearningValidation;
      }

    protected:
      /// Protected constructor
      /// Users need to call RbPrmPlanner::create in order to create instances.
      LearningPathValidation::LearningPathValidation(const core::DevicePtr_t &robot, const core::value_type &stepSize) :
          RbPrmPathValidation(robot,stepSize)
      {}

    private:
      LearningValidationPtr_t learningValidation_;

    }; // class LearningPathValidation


  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_LEARNING_PATH_VALIDATION_HH
