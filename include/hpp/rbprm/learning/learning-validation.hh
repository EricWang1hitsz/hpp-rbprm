//
// Copyright (c) 2017 CNRS
// Authors: Mathieu Geisert
//          (adapted from Pierre Fernbach's code: dynamic-validation.hh)
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

#ifndef HPP_RBPRM_LEARNING_VALIDATION_HH
# define HPP_RBPRM_LEARNING_VALIDATION_HH

#include <hpp/core/config-validation.hh>
#include <hpp/rbprm/rbprm-validation-report.hh>
#include <hpp/rbprm/learning/GMM.hh>

namespace hpp {
  namespace rbprm {


    /// Exception thrown when a configuration is not within the bounds
    class LearningValidationReport : public core::ValidationReport
    {
    public:
      LearningValidationReport () : ValidationReport () {}
      /// Print report in a stream
      virtual std::ostream& print (std::ostream& os) const
      {
        os << "Learning report...";
        return os;
      }
    }; // class LearningValidationReport



    HPP_PREDEF_CLASS(LearningValidation);
    typedef boost::shared_ptr <LearningValidation> LearningValidationPtr_t;

    class LearningValidation : public core::ConfigValidation
    {
    public:
      static LearningValidationPtr_t LearningValidation::create (GMM gmm)
      {
        LearningValidation* ptr = new LearningValidation (gmm);
        return LearningValidationPtr_t (ptr);
      }
      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \retval validationReport report on validation. Must be a valid rbprmReport with the latest collision informations.
      ///         If non valid, a new validation report will be allocated
      ///         and returned via this shared pointer.
      /// \return whether the whole config is valid.
      virtual bool validate (const core::Configuration_t& config, core::ValidationReportPtr_t& validationReport);

      void setInitialReport(core::ValidationReportPtr_t initialReport);

    protected:
      LearningValidation (GMM gmm);
    private:
      GMM gmm_;
      core::RbprmValidationReportPtr_t lastReport_;
      bool initContacts_;
    }; // class LearningValidation


  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_LEARNING_VALIDATION_HH
