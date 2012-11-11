#include "common/Plugin.hh"
#include "gazebo.hh"

namespace gazebo {
  class PublishScene : public ModelPlugin {
    public:
      PublishScene() : ModelPlugin() {
      }

      void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf){
      }
  };
  GZ_REGISTER_MODEL_PLUGIN(PublishScene)
}
