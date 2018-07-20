#ifndef HPP_LEARNING_NODE_HH
#define HPP_LEARNING_NODE_HH

#include <hpp/core/node.hh>
#include <hpp/rbprm/rbprm-validation-report.hh>
#include <centroidal-dynamics-lib/centroidal_dynamics.hh>
#include <boost/tuple/tuple.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>
#include <hpp/model/configuration.hh>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/intersect.h>
#include <hpp/rbprm/planner/rbprm-node.hh>

#include <hpp/rbprm/learning/learning-validation.hh>

namespace hpp {
  namespace core {

    HPP_PREDEF_CLASS (LearningNode);
    typedef boost::shared_ptr <LearningNode> LearningNodePtr_t;


    class HPP_CORE_DLLAPI LearningNode : public Node
    {
    public :
      /// Constructor
      /// \param configuration configuration stored in the new node
      /// \note A new connected component is created. For consistency, the
      ///       new node is not registered in the connected component.
      LearningNode (const ConfigurationPtr_t& configuration):
        Node(configuration)
      {}
      /// Constructor
      /// \param configuration configuration stored in the new node
      /// \param connectedComponent connected component the node belongs to.
      LearningNode (const ConfigurationPtr_t& configuration,
      ConnectedComponentPtr_t connectedComponent):
        Node(configuration,connectedComponent)
      {}

      void setMap(const std::map<std::string, hpp::rbprm::SurfaceDatas_t> map)
      {
          ROMSurfacesMap_ = map;
      }

      hpp::rbprm::SurfaceData getContactSurfaceFromROM(const std::string& romName)
      {
        hpp::rbprm::SurfaceDatas_t surfaces = ROMSurfacesMap_[romName];
        return surfaces.front();
      }

      bool addContactSurfaceFromROM(const std::string& romName, hpp::rbprm::SurfaceDatas_t contactSurface);

      RbprmValidationReportPtr_t getReport(){
        return collisionReport_;
      }

      void collisionReport(RbprmValidationReportPtr_t report){
        collisionReport_ = report;
      }

      void setNumberOfContacts(int n){numberOfContacts_ = n;}
      int getNumberOfContacts(){return numberOfContacts_;}

      double getScore(){return score_;}
      void setScore(const double& score){score_ = score;}

    private:
      int numberOfContacts_;
      std::map<std::string, hpp::rbprm::SurfaceDatas_t> ROMSurfacesMap_; //id Rom, support surface
      double score_;
      RbprmValidationReportPtr_t collisionReport_;

    //  const polytope::ProjectedCone* giwc_; // useless now ?
    //  polytope::T_rotation_t rotContact_;
    //  polytope::vector_t posContact_;


    }; // class

  }//core
}//hpp

#endif // HPP_LEARNING_NODE_HH
