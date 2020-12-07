#ifndef KEYFRAME_TOOL_H
#define KEYFRAME_TOOL_H

#include <rviz/tool.h>

namespace Ogre
{
class SceneNode;
class Vector3;
class Quaternion;
}

namespace rviz
{
class Property;
class StringProperty;
class BoolProperty;
class VectorProperty;
class QuaternionProperty;
class FloatProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace rviz_animator
{

class KeyframeTool: public rviz::Tool
{
Q_OBJECT
public:
  KeyframeTool();
  ~KeyframeTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

private Q_SLOTS:
  void updateKeyframePosition(int keyframe_index);
  void updateKeyframeOrientation(int keyframe_index);

private:
  void makeKeyframe( const Ogre::Vector3& position, const Ogre::Quaternion& orientation );

  std::vector<Ogre::SceneNode*> keyframe_nodes_;
  std::string keyframe_resource_;
  rviz::Property* keyframes_property_;
  rviz::StringProperty* topic_property_;
  rviz::BoolProperty* publish_property_;

};

} // end namespace rviz_animator

#endif // KEYFRAME_TOOL_H

