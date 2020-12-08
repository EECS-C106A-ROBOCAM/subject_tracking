#include <OGRE/OgreCamera.h>

#include "AnimationViewController.h"

namespace rviz_animator
{

AnimationViewController::AnimationViewController() : FPSViewController() {}

AnimationViewController::~AnimationViewController() {}

void AnimationViewController::onInitialize()
{
    FPSViewController::onInitialize();
}

void AnimationViewController::setPose(Ogre::Vector3 position, Ogre::Quaternion orientation) {
    camera_->setPosition(position);
    camera_->setOrientation(orientation);
}

} // end namespace rviz_animators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_animator::AnimationViewController, rviz::FPSViewController)