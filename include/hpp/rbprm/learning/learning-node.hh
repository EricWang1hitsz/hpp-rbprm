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

namespace hpp {
  namespace core {

    HPP_PREDEF_CLASS (LearningNode);
    typedef LearningNode* LearningNodePtr_t;

    //Triplet collision object, surface normal, contact point.
    typedef boost::tuple<CollisionObjectPtr_t, geom::Point , geom::Point> ContactSurface;

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
      RbprmNode (const ConfigurationPtr_t& configuration,
      ConnectedComponentPtr_t connectedComponent):
        Node(configuration,connectedComponent)
      {}

      ContactSurface getContactSurfaceFromROM(const std::string& romName)
      {
        return ROMSurfacesMap_[romName];
      }

      bool addContactSurfaceFromROM(const std::string& romName, ContactSurface contactSurface, bool replace=false)
      {
        bool success = ROMSurfacesMap_.insert(std::pair<char,int>(romName, contactSurface));
        if(success==true)
        {
            numberOfContacts_++;
        }
        else if (replace==true)
        {
            ROMSurfacesMap_.find(romName)->second = contactSurface;
            success = true;
        }
        else
        {
            hppDout(info, "ROM " << romName << " is already in the map. Please use replaceConctactSurfaceFromROM instead.");
        }
        return success;
      }

      ContactSurface addContactSurfaceFromROM(const std::string& romName, const CollisionObjectPtr_t& collisionObject, const geom::Point& normal, const geom::Point& contactPoint)
      {
        return addContactSurfaceFromROM(romName, ContactSurface(collisionObject, normal, contactPoint));
      }

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
      std::map<std::string, ContactSurface> ROMSurfacesMap_; //id Rom, support surface
      double score_;
      RbprmValidationReportPtr_t collisionReport_;

    //  const polytope::ProjectedCone* giwc_; // useless now ?
    //  polytope::T_rotation_t rotContact_;
    //  polytope::vector_t posContact_;


    }; // class

  }//core
}//hpp

#endif // HPP_LEARNING_NODE_HH
