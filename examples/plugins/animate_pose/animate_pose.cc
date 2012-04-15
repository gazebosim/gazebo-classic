#include "common/CommonTypes.hh"
#include "common/Animation.hh"
#include "common/KeyFrame.hh"
#include "physics/Model.hh"
#include "gazebo.h"

namespace gazebo
{
  class AnimatePose : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      gazebo::common::PoseAnimationPtr anim(
          new gazebo::common::PoseAnimation("test", 1000.0, true));

      gazebo::common::PoseKeyFrame *key;

      key = anim->CreateKeyFrame(0);
      key->SetTranslation(math::Vector3(0, 0, 0));
      key->SetRotation(math::Quaternion(0, 0, 0));

      key = anim->CreateKeyFrame(1000.0);
      key->SetTranslation(math::Vector3(5, 0, 0));
      key->SetRotation(math::Quaternion(0, 0, 1.5707));

      _parent->SetAnimation(anim);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatePose)
}
