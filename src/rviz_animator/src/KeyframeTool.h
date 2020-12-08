#ifndef KEYFRAME_TOOL_H
#define KEYFRAME_TOOL_H

#include <ros/ros.h>

#include <rviz/tool.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

namespace rviz_animator
{

struct KeyframeStruct {
  Ogre::SceneNode* node_;
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
  float timestamp_;
  int id_;
  std::string label_;
  bool operator< (const KeyframeStruct& other) {return timestamp_ < other.timestamp_;}
};

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
  void updateKeyframePosition(int keyframe_id);
  void updateKeyframeOrientation(int keyframe_id);
  void updateKeyframeTimestamp(int keyframe_id);
  void updateKeyframeLabel(int property_index);
  void deleteKeyframe(int property_index);
  void previewKeyframes();
  void publishKeyframes();

private:
  Ogre::SceneNode* createNode( const Ogre::Vector3& position, const Ogre::Quaternion& orientation );
  void sortKeyframes();
  void renderKeyframeProperties();
  
  ros::Publisher pub_;
  std::vector<KeyframeStruct> keyframes_;
  std::string keyframe_resource_;
  rviz::Property* keyframes_property_;
  rviz::StringProperty* topic_property_;
  rviz::BoolProperty* publish_property_;
  rviz::BoolProperty* preview_property_;
  static int current_keyframe_id;
};

} // end namespace rviz_animator

#endif // KEYFRAME_TOOL_H

