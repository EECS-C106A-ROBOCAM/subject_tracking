#include <iostream>
#include <algorithm>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>

#include "keyframe_tool.h"

namespace rviz_animator
{

KeyframeTool::KeyframeTool()
{
  shortcut_key_ = 'k';
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
  keyframe_resource_ = "package://rviz_plugin_tutorials/media/flag.dae";

  if( rviz::loadMeshFromResource( keyframe_resource_ ).isNull() )
  {
    ROS_ERROR( "KeyframeTool: failed to load model resource '%s'.", keyframe_resource_.c_str() );
    return;
  }

  keyframes_property_ = new rviz::Property("Keyframes");
  topic_property_ = new rviz::StringProperty("Topic", "/operator_keyframes");
  publish_property_ = new rviz::BoolProperty("Publish?");

  getPropertyContainer()->addChild( keyframes_property_ );
  getPropertyContainer()->addChild( topic_property_ );
  getPropertyContainer()->addChild( publish_property_ );
}

void KeyframeTool::activate() {}

void KeyframeTool::deactivate() {}

void KeyframeTool::updateKeyframePosition(int property_index)
{
  auto& keyframe = keyframes_[property_index];
  keyframe.position_ = dynamic_cast<rviz::VectorProperty*>(keyframes_property_->childAt(property_index)->childAt(0))->getVector();
  keyframe.node_->setPosition(keyframe.position_);
}

void KeyframeTool::updateKeyframeOrientation(int property_index)
{
  auto& keyframe = keyframes_[property_index];
  keyframe.orientation_ = dynamic_cast<rviz::QuaternionProperty*>(keyframes_property_->childAt(property_index)->childAt(1))->getQuaternion();
  keyframe.node_->setOrientation(keyframe.orientation_);
}


void KeyframeTool::updateKeyframeTimestamp(int property_index)
{
  auto& keyframe = keyframes_[property_index];
  keyframe.timestamp_ = dynamic_cast<rviz::FloatProperty*>(keyframes_property_->childAt(property_index)->childAt(2))->getFloat();
  sortKeyframes();
}

void KeyframeTool::updateKeyframeLabel(int property_index)
{
  auto& keyframe = keyframes_[property_index];
  keyframe.label_ = dynamic_cast<rviz::StringProperty*>(keyframes_property_->childAt(property_index)->childAt(3))->getStdString();
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

    dynamic_cast<rviz::VectorProperty*>(keyframe_property->childAt(0))->setVector(keyframe.position_);
    dynamic_cast<rviz::QuaternionProperty*>(keyframe_property->childAt(1))->setQuaternion(keyframe.orientation_);
    dynamic_cast<rviz::FloatProperty*>(keyframe_property->childAt(2))->setFloat(keyframe.timestamp_);
    dynamic_cast<rviz::StringProperty*>(keyframe_property->childAt(3))->setStdString(keyframe.label_);
  }
}

int KeyframeTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  Ogre::Camera* camera = scene_manager_->getCurrentViewport()->getCamera();
  auto node = createNode(camera->getPosition(), camera->getOrientation());
  auto keyframe = Keyframe{node, camera->getPosition(), camera->getOrientation(), 0, current_keyframe_id, "keyframe_id_" + std::to_string(current_keyframe_id)};
  keyframes_.push_back(keyframe);
  current_keyframe_id++;

  rviz::Property* keyframe_property = new rviz::Property("Keyframe " + QString::number(keyframes_property_->numChildren()));
  rviz::VectorProperty* keyframe_position_property = new rviz::VectorProperty("Position");
  rviz::QuaternionProperty* keyframe_orientation_property = new rviz::QuaternionProperty("Orientation");
  rviz::FloatProperty* keyframe_timestamp_property = new rviz::FloatProperty("Timestamp");
  rviz::StringProperty* keyframe_label_property = new rviz::StringProperty("Label");

  keyframe_position_property ->setVector(keyframe.position_);
  keyframe_orientation_property->setQuaternion(keyframe.orientation_);
  keyframe_timestamp_property->setFloat(keyframe.timestamp_);
  keyframe_label_property->setStdString(keyframe.label_);

  int property_index = keyframes_property_->numChildren();
  connect(keyframe_position_property, &rviz::VectorProperty::changed, this, [=](){ updateKeyframePosition(property_index); });
  connect(keyframe_orientation_property, &rviz::QuaternionProperty::changed, this, [=](){ updateKeyframeOrientation(property_index); });
  connect(keyframe_timestamp_property, &rviz::FloatProperty::changed, this, [=](){ updateKeyframeTimestamp(property_index); });
  connect(keyframe_label_property, &rviz::StringProperty::changed, this, [=](){ updateKeyframeLabel(property_index); });

  keyframes_property_->addChild( keyframe_property );
  keyframe_property->addChild( keyframe_position_property );
  keyframe_property->addChild( keyframe_orientation_property );
  keyframe_property->addChild( keyframe_timestamp_property );
  keyframe_property->addChild( keyframe_label_property );

  return Render | Finished;
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

void KeyframeTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );
  rviz::Config keyframe_tool_config = config.mapMakeChild( "Keyframe Tool" );
  rviz::Config keyframe_topic_config = keyframe_tool_config.mapMakeChild("Topic");
  topic_property_->save( keyframe_topic_config );

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

    keyframe_position_property->load( keyframe_config.mapGetChild("Position") );
    keyframe_orientation_property->load( keyframe_config.mapGetChild("Orientation") );
    keyframe_timestamp_property->load( keyframe_config.mapGetChild("Timestamp") );

    keyframe_property->addChild( keyframe_position_property );
    keyframe_property->addChild( keyframe_orientation_property );
    keyframe_property->addChild( keyframe_timestamp_property );

    keyframes_property_->addChild( keyframe_property );

    auto node = createNode( keyframe_position_property->getVector(), keyframe_orientation_property->getQuaternion() );
    // keyframes_.push_back(Keyframe{node, keyframe_timestamp_property->getFloat(), current_keyframe_id++, current_keyframe_id});
  }
}

int KeyframeTool::current_keyframe_id = 0;

} // end namespace rviz_animator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_animator::KeyframeTool,rviz::Tool )