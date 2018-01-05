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
#include <hpp/rbprm/rbprm-validation.hh>

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

class LearningValidation : public rbprm::RbPrmValidation
{
    typedef rbprm::RbPrmValidation parent_t;

public:

    static LearningValidationPtr_t create (GMMPtr_t gmm,
                                           const model::RbPrmDevicePtr_t& robot,
                                           const std::vector<std::string>& filter = std::vector<std::string>(),
                                           const std::map<std::string, std::vector<std::string> >& affFilters =  std::map<std::string, std::vector<std::string> >(),
                                           const std::map<std::string, std::vector<model::CollisionObjectPtr_t> >& affordances = std::map<std::string,                      std::vector<model::CollisionObjectPtr_t> >(),
                                           const core::ObjectVector_t& geometries = core::ObjectVector_t());


    /// \param config the config to check for validity,
    /// \param filter specify constraints on all roms required to be in contact, will return
    /// \param validationReport the report (can be cast to rbprmValidationReport) with info on the trunk and ROM states,
    /// \return whether the whole config is valid.
    virtual bool validateRoms(const core::Configuration_t& config,
                      const std::vector<std::string>& filter,
                       core::ValidationReportPtr_t &validationReport);

protected:

    LearningValidation (GMMPtr_t gmm,
                        const model::RbPrmDevicePtr_t& robot,
                        const std::vector<std::string>& filter,
                        const std::map<std::string,
                        std::vector<std::string> >& affFilters,
                        const std::map<std::string,
                        std::vector<model::CollisionObjectPtr_t> >& affordances,
                        const core::ObjectVector_t& geometries);

private:
    GMMPtr_t gmm_;

}; // class LearningValidation


} // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_LEARNING_VALIDATION_HH
