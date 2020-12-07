#include <iostream>
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
  for( unsigned i = 0; i < keyframe_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( keyframe_nodes_[ i ]);
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

void KeyframeTool::activate()
{

}

void KeyframeTool::deactivate()
{

}

void KeyframeTool::updateKeyframePosition(int keyframe_index)
{
  if (keyframe_index >= keyframe_nodes_.size()) return;
  keyframe_nodes_[keyframe_index]->setPosition(dynamic_cast<rviz::VectorProperty*>(keyframes_property_->childAt(keyframe_index)->childAt(0))->getVector());
}

void KeyframeTool::updateKeyframeOrientation(int keyframe_index)
{
  if (keyframe_index >= keyframe_nodes_.size()) return;
  keyframe_nodes_[keyframe_index]->setOrientation(dynamic_cast<rviz::QuaternionProperty*>(keyframes_property_->childAt(keyframe_index)->childAt(1))->getQuaternion());
}

int KeyframeTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{

  rviz::Property* keyframe_property = new rviz::Property("Keyframe " + QString::number( keyframe_nodes_.size()));
  rviz::VectorProperty* keyframe_position_property = new rviz::VectorProperty("Position");
  rviz::QuaternionProperty* keyframe_orientation_property = new rviz::QuaternionProperty("Orientation");
  rviz::FloatProperty* keyframe_timestamp_property = new rviz::FloatProperty("Timestamp");
 
  int keyframe_index = keyframe_nodes_.size();
  connect(keyframe_position_property, &rviz::VectorProperty::changed, this, [=](){ updateKeyframePosition(keyframe_index); });
  connect(keyframe_orientation_property, &rviz::QuaternionProperty::changed, this, [=](){ updateKeyframeOrientation(keyframe_index); });

  keyframes_property_->addChild( keyframe_property );
  keyframe_property->addChild( keyframe_position_property );
  keyframe_property->addChild( keyframe_orientation_property );
  keyframe_property->addChild( keyframe_timestamp_property );

  Ogre::Camera* camera = scene_manager_->getCurrentViewport()->getCamera();

  keyframe_position_property ->setVector(camera->getPosition());
  keyframe_orientation_property->setQuaternion(camera->getOrientation());
  keyframe_timestamp_property->setValue(0);

  makeKeyframe(camera->getPosition(), camera->getOrientation());

  return Render | Finished;

}

void KeyframeTool::makeKeyframe( const Ogre::Vector3& position, const Ogre::Quaternion& orientation )
{
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( keyframe_resource_ );
  Ogre::Camera* camera = scene_manager_->getCurrentViewport()->getCamera();

  node->attachObject( entity );
  node->setPosition(position);
  node->setOrientation(orientation);
  node->setVisible( true );
  keyframe_nodes_.push_back( node );
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

    makeKeyframe( keyframe_position_property->getVector(), keyframe_orientation_property->getQuaternion() );
  }
}


} // end namespace rviz_animator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_animator::KeyframeTool,rviz::Tool )