#ifndef ANIMATION_VIEW_CONTROLLER_H
#define ANIMATION_VIEW_CONTROLLER_H

#include <OGRE/OgreEntity.h>

#include <rviz/default_plugin/view_controllers/fps_view_controller.h>

namespace rviz_animator
{

class AnimationViewController: public rviz::FPSViewController
{
Q_OBJECT
public:
  AnimationViewController();
  ~AnimationViewController();

  void onInitialize();
  void setPose(Ogre::Vector3 position, Ogre::Quaternion orientation);
};

} // end namespace rviz_animator


#endif // ANIMATION_VIEW_CONTROLLER_H