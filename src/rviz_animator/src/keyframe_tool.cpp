#include <iostream>
#include <algorithm>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>

#include "keyframe_tool.h"
#include "rviz_animator/KeyframeMsg.h"
#include "rviz_animator/KeyframesMsg.h"

namespace rviz_animator
{

int KeyframeTool::current_keyframe_id = 0;

KeyframeTool::KeyframeTool()
{
  ros::NodeHandle nh;
  shortcut_key_ = 'k';
  pub_ = nh.advertise<KeyframesMsg>("/operator_keyframes", 1);
}

KeyframeTool::~KeyframeTool()
{
  for (auto keyframe : keyframes_)
  {
    scene_manager_->destroySceneNode(keyframe.node_);
  }
}

void KeyframeTool::onInitialize()
{
  // keyframe_resource_ = "package://rviz_animator/media/camera.dae";
  keyframe_resource_ = "package://rviz_plugin_tutorials/media/flag.dae";

  if( rviz::loadMeshFromResource( keyframe_resource_ ).isNull() )
  {
    ROS_ERROR( "KeyframeTool: failed to load model resource '%s'.", keyframe_resource_.c_str() );
    return;
  }

  keyframes_property_ = new rviz::Property("Keyframes");
  topic_property_ = new rviz::StringProperty("Topic", "/operator_keyframes");
  publish_property_ = new rviz::BoolProperty("Publish?", false);
  preview_property_ = new rviz::BoolProperty("Preview?", false);

  getPropertyContainer()->addChild( topic_property_ );
  // getPropertyContainer()->addChild( preview_property_ ); // TODO(JS): Fix
  getPropertyContainer()->addChild( publish_property_ );
  getPropertyContainer()->addChild( keyframes_property_ );

  connect(preview_property_, &rviz::BoolProperty::changed, this, &KeyframeTool::previewKeyframes);
  connect(publish_property_, &rviz::BoolProperty::changed, this, &KeyframeTool::publishKeyframes);

}

void KeyframeTool::activate() {}

void KeyframeTool::deactivate() {}

int KeyframeTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  Ogre::Camera* camera = scene_manager_->getCurrentViewport()->getCamera();
  auto node = createNode(camera->getPosition(), camera->getOrientation());
  auto keyframe = KeyframeStruct{node, camera->getPosition(), camera->getOrientation(), 0, current_keyframe_id, "keyframe_id_" + std::to_string(current_keyframe_id)};
  keyframes_.push_back(keyframe);
  current_keyframe_id++;

  rviz::Property* keyframe_property = new rviz::Property("Keyframe " + QString::number(keyframes_property_->numChildren()));
  rviz::BoolProperty* keyframe_active_property = new rviz::BoolProperty("Active?", true, "", keyframe_property);
  rviz::IntProperty* keyframe_id_property = new rviz::IntProperty("ID", keyframe.id_, "", keyframe_property);
  rviz::StringProperty* keyframe_label_property = new rviz::StringProperty("Label", QString::fromStdString(keyframe.label_), "", keyframe_property);
  rviz::VectorProperty* keyframe_position_property = new rviz::VectorProperty("Position", keyframe.position_, "", keyframe_property);
  rviz::QuaternionProperty* keyframe_orientation_property = new rviz::QuaternionProperty("Orientation", keyframe.orientation_, "", keyframe_property);
  rviz::FloatProperty* keyframe_timestamp_property = new rviz::FloatProperty("Timestamp", keyframe.timestamp_, "", keyframe_property);

  int property_index = keyframes_property_->numChildren();
  connect(keyframe_position_property, &rviz::VectorProperty::changed, this, [=](){ updateKeyframePosition(property_index); });
  connect(keyframe_orientation_property, &rviz::QuaternionProperty::changed, this, [=](){ updateKeyframeOrientation(property_index); });
  connect(keyframe_timestamp_property, &rviz::FloatProperty::changed, this, [=](){ updateKeyframeTimestamp(property_index); });
  connect(keyframe_label_property, &rviz::StringProperty::changed, this, [=](){ updateKeyframeLabel(property_index); });
  connect(keyframe_active_property, &rviz::BoolProperty::changed, this, [=](){ deleteKeyframe(property_index); });

  keyframe_id_property->setReadOnly(true);
  keyframes_property_->addChild( keyframe_property );

  return Render | Finished;
}

void KeyframeTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );
  rviz::Config keyframe_tool_config = config.mapMakeChild( "Keyframe Tool" );
  rviz::Config keyframe_topic_config = keyframe_tool_config.mapMakeChild("Topic");
  topic_property_->save( keyframe_topic_config );
  
  keyframe_tool_config.mapSetValue("current_keyframe_id", current_keyframe_id);

  rviz::Config keyframes_config = keyframe_tool_config.mapMakeChild( "Keyframes" );

  int num_children = keyframes_property_->numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    rviz::Property* keyframe_prop = keyframes_property_->childAt( i );
    rviz::Config keyframe_config = keyframes_config.listAppendNew();

    keyframe_config.mapSetValue( "Name", keyframe_prop->getName() );
    for (int j = 0; j < keyframe_prop->numChildren(); j++) {
      rviz::Property* keyframe_aspect_prop = keyframe_prop->childAt(j);
      rviz::Config keyframe_aspect_config = keyframe_config.mapMakeChild(keyframe_aspect_prop->getName());
      keyframe_aspect_prop->save( keyframe_aspect_config );
    }
  }
}

void KeyframeTool::load( const rviz::Config& config )
{
  rviz::Config keyframe_tool_config = config.mapGetChild( "Keyframe Tool" );
  rviz::Config keyframe_topic_config = keyframe_tool_config.mapGetChild("Topic");
  keyframe_tool_config.mapGetInt("current_keyframe_id", &current_keyframe_id);

  topic_property_->load(keyframe_topic_config);
  
  rviz::Config keyframes_config = keyframe_tool_config.mapGetChild( "Keyframes" );

  int num_keyframes = keyframes_config.listLength();
  for( int i = 0; i < num_keyframes; i++ )
  {
    rviz::Config keyframe_config = keyframes_config.listChildAt( i );
    QString name = "Keyframe " + QString::number( i );
    keyframe_config.mapGetString( "Name", &name );

    rviz::Property* keyframe_property = new rviz::Property(name);
    rviz::VectorProperty* keyframe_position_property = new rviz::VectorProperty("Position");
    rviz::QuaternionProperty* keyframe_orientation_property = new rviz::QuaternionProperty("Orientation");
    rviz::FloatProperty* keyframe_timestamp_property = new rviz::FloatProperty("Timestamp");
    rviz::StringProperty* keyframe_label_property = new rviz::StringProperty("Label");
    rviz::IntProperty* keyframe_id_property = new rviz::IntProperty("ID");
    rviz::BoolProperty* keyframe_active_property = new rviz::BoolProperty("Active?", true);
    

    keyframe_position_property->load( keyframe_config.mapGetChild("Position") );
    keyframe_orientation_property->load( keyframe_config.mapGetChild("Orientation") );
    keyframe_timestamp_property->load( keyframe_config.mapGetChild("Timestamp") );
    keyframe_label_property->load( keyframe_config.mapGetChild("Label") );
    keyframe_id_property->load( keyframe_config.mapGetChild("ID") );

    int property_index = keyframes_property_->numChildren();
    connect(keyframe_position_property, &rviz::VectorProperty::changed, this, [=](){ updateKeyframePosition(property_index); });
    connect(keyframe_orientation_property, &rviz::QuaternionProperty::changed, this, [=](){ updateKeyframeOrientation(property_index); });
    connect(keyframe_timestamp_property, &rviz::FloatProperty::changed, this, [=](){ updateKeyframeTimestamp(property_index); });
    connect(keyframe_label_property, &rviz::StringProperty::changed, this, [=](){ updateKeyframeLabel(property_index); });
    connect(keyframe_active_property, &rviz::BoolProperty::changed, this, [=](){ deleteKeyframe(property_index); });    

    keyframe_property->addChild( keyframe_active_property );
    keyframe_property->addChild( keyframe_id_property );
    keyframe_property->addChild( keyframe_label_property );
    keyframe_property->addChild( keyframe_position_property );
    keyframe_property->addChild( keyframe_orientation_property );
    keyframe_property->addChild( keyframe_timestamp_property );
    keyframes_property_->addChild( keyframe_property );

    auto node = createNode( keyframe_position_property->getVector(), keyframe_orientation_property->getQuaternion() );
    auto keyframe = KeyframeStruct{node, keyframe_position_property->getVector(), keyframe_orientation_property->getQuaternion(),
                              keyframe_timestamp_property->getFloat(), keyframe_id_property->getInt(), keyframe_label_property->getStdString()};
    keyframes_.push_back(keyframe);
  }
}

void KeyframeTool::sortKeyframes()
{
  std::sort(keyframes_.begin(), keyframes_.end());
  renderKeyframeProperties();
}

void KeyframeTool::renderKeyframeProperties() {
  for (int i = 0; i < keyframes_.size(); ++i) {
    auto keyframe = keyframes_[i];
    auto keyframe_property = keyframes_property_->childAt(i);

    dynamic_cast<rviz::VectorProperty*>(keyframe_property->subProp("Position"))->setVector(keyframe.position_);
    dynamic_cast<rviz::QuaternionProperty*>(keyframe_property->subProp("Orientation"))->setQuaternion(keyframe.orientation_);
    dynamic_cast<rviz::FloatProperty*>(keyframe_property->subProp("Timestamp"))->setFloat(keyframe.timestamp_);
    dynamic_cast<rviz::StringProperty*>(keyframe_property->subProp("Label"))->setStdString(keyframe.label_);
    dynamic_cast<rviz::IntProperty*>(keyframe_property->subProp("ID"))->setInt(keyframe.id_);
    dynamic_cast<rviz::BoolProperty*>(keyframe_property->subProp("Active?"))->setBool(true);
  }
}

void KeyframeTool::previewKeyframes() {
  if (!preview_property_->getBool()) return;
  context_->getViewManager()->setCurrentViewControllerType("rviz/FPS");
  auto view_controller = context_->getViewManager()->getCurrent();
  auto keyframe = keyframes_[0];
  view_controller->subProp("Position")->subProp("X")->setValue(keyframe.position_[0]);
  view_controller->subProp("Position")->subProp("Y")->setValue(keyframe.position_[1]);
  view_controller->subProp("Position")->subProp("Z")->setValue(keyframe.position_[2]);

}

void KeyframeTool::publishKeyframes() {
  if (!publish_property_->getBool()) return;
  KeyframesMsg keyframes_msg;
  keyframes_msg.keyframes.clear();
  keyframes_msg.num_keyframes = keyframes_.size();
  for (int i = 0; i < keyframes_.size(); ++i) {
    KeyframeStruct keyframe = keyframes_[i];
    KeyframeMsg keyframe_msg;
    keyframe_msg.frame.position.x = keyframe.position_[0];
    keyframe_msg.frame.position.y = keyframe.position_[1];
    keyframe_msg.frame.position.z = keyframe.position_[2];
    keyframe_msg.frame.orientation.x = keyframe.orientation_[0];
    keyframe_msg.frame.orientation.y = keyframe.orientation_[1];
    keyframe_msg.frame.orientation.z = keyframe.orientation_[2];
    keyframe_msg.frame.orientation.w = keyframe.orientation_[3];
    keyframe_msg.timestamp = keyframe.timestamp_;
    keyframes_msg.keyframes.push_back(keyframe_msg);
  }
  pub_.publish(keyframes_msg);
}

void KeyframeTool::updateKeyframePosition(int property_index)
{
  auto& keyframe = keyframes_[property_index];
  keyframe.position_ = dynamic_cast<rviz::VectorProperty*>(keyframes_property_->childAt(property_index)->subProp("Position"))->getVector();
  keyframe.node_->setPosition(keyframe.position_);
}

void KeyframeTool::updateKeyframeOrientation(int property_index)
{
  auto& keyframe = keyframes_[property_index];
  keyframe.orientation_ = dynamic_cast<rviz::QuaternionProperty*>(keyframes_property_->childAt(property_index)->subProp("Orientation"))->getQuaternion();
  keyframe.node_->setOrientation(keyframe.orientation_);
}


void KeyframeTool::updateKeyframeTimestamp(int property_index)
{
  auto& keyframe = keyframes_[property_index];
  keyframe.timestamp_ = dynamic_cast<rviz::FloatProperty*>(keyframes_property_->childAt(property_index)->subProp("Timestamp"))->getFloat();
  sortKeyframes();
}

void KeyframeTool::updateKeyframeLabel(int property_index)
{
  auto& keyframe = keyframes_[property_index];
  keyframe.label_ = dynamic_cast<rviz::StringProperty*>(keyframes_property_->childAt(property_index)->subProp("Label"))->getStdString();
}

void KeyframeTool::deleteKeyframe(int property_index) {
  if (keyframes_property_->childAt(property_index)->subProp("Active?")->getValue().toBool()) return;
  auto& keyframe = keyframes_[property_index];
  scene_manager_->destroySceneNode(keyframe.node_);
  keyframes_.erase(keyframes_.begin() + property_index);
  keyframes_property_->removeChildren(keyframes_property_->numChildren() - 1, 1);
  renderKeyframeProperties();
}

Ogre::SceneNode* KeyframeTool::createNode( const Ogre::Vector3& position, const Ogre::Quaternion& orientation )
{
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( keyframe_resource_ );
  Ogre::Camera* camera = scene_manager_->getCurrentViewport()->getCamera();

  node->attachObject( entity );
  node->setPosition(position);
  node->setOrientation(orientation);
  node->setVisible( true );

  return node;
}

} // end namespace rviz_animator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_animator::KeyframeTool,rviz::Tool )