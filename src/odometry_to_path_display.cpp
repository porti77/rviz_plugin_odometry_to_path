/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/bind.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMatrix4.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"

#include "rviz/ogre_helpers/billboard_line.h"
#include "odometry_to_path_display.h"

namespace rviz_plugin_odometry_to_path
{

OdometryToPathDisplay::OdometryToPathDisplay()
{
  style_property_ = new rviz::EnumProperty( "Line Style", "Lines",
                                      "The rendering operation to use to draw the grid lines.",
                                      this, SLOT( updateStyle() ));

  style_property_->addOption( "Lines", LINES );
  style_property_->addOption( "Billboards", BILLBOARDS );

  line_width_property_ = new rviz::FloatProperty( "Line Width", 0.5,
                                            "The width, in meters, of each path line."
                                            "Only works with the 'Billboards' style.",
                                            this, SLOT( updateLineWidth() ), this );
  line_width_property_->setMin( 0.001 );
  line_width_property_->hide();

  color_property_ = new rviz::ColorProperty( "Color", QColor( 25, 255, 0 ),
                                       "Color to draw the path.", this );

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                       "Amount of transparency to apply to the path.", this );

  buffer_length_property_ = new rviz::IntProperty( "Buffer Length", 1000,
                                             "Number of paths to display.",
                                             this, SLOT( updateBufferLength() ));
  buffer_length_property_->setMin( 1 );

  offset_property_ = new rviz::VectorProperty( "Offset", Ogre::Vector3::ZERO,
                                         "Allows you to offset the path from the origin of the reference frame.  In meters.",
                                         this, SLOT( updateOffset() ));

  first_time_ = true;
}

OdometryToPathDisplay::~OdometryToPathDisplay()
{
  destroyObjects();
}

void OdometryToPathDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateBufferLength();
}

void OdometryToPathDisplay::reset()
{
  MFDClass::reset();
  updateBufferLength();
}


void OdometryToPathDisplay::updateStyle()
{
  LineStyle style = (LineStyle) style_property_->getOptionInt();

  switch( style )
  {
  case LINES:
  default:
    line_width_property_->hide();
    break;

  case BILLBOARDS:
    line_width_property_->show();
    break;
  }

  updateBufferLength();
}

void OdometryToPathDisplay::updateLineWidth()
{
  LineStyle style = (LineStyle) style_property_->getOptionInt();
  float line_width = line_width_property_->getFloat();

  if(style == BILLBOARDS) {
    for( size_t i = 0; i < billboard_lines_.size(); i++ )
    {
      rviz::BillboardLine* billboard_line = billboard_lines_[ i ];
      if( billboard_line ) billboard_line->setLineWidth( line_width );
    }
  }
  context_->queueRender();
}

void OdometryToPathDisplay::updateOffset()
{
  scene_node_->setPosition( offset_property_->getVector() );
  context_->queueRender();
}

void OdometryToPathDisplay::destroyObjects()
{
  // Destroy all simple lines, if any
  for( size_t i = 0; i < manual_objects_.size(); i++ )
  {
    Ogre::ManualObject*& manual_object = manual_objects_[ i ];
    if( manual_object )
    {
      manual_object->clear();
      scene_manager_->destroyManualObject( manual_object );
      manual_object = NULL; // ensure it doesn't get destroyed again
    }
  }

  // Destroy all billboards, if any
  for( size_t i = 0; i < billboard_lines_.size(); i++ )
  {
    rviz::BillboardLine*& billboard_line = billboard_lines_[ i ];
    if( billboard_line )
    {
      delete billboard_line; // also destroys the corresponding scene node
      billboard_line = NULL; // ensure it doesn't get destroyed again
    }
  }
}

void OdometryToPathDisplay::updateBufferLength()
{
  // Delete old path objects
  destroyObjects();

  // Read options
  int buffer_length = buffer_length_property_->getInt();
  LineStyle style = (LineStyle) style_property_->getOptionInt();

  // Create new path objects
  switch(style)
  {
  case LINES: // simple lines with fixed width of 1px
    manual_objects_.resize( buffer_length );
    for( size_t i = 0; i < manual_objects_.size(); i++ )
    {
      Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
      manual_object->setDynamic( true );
      scene_node_->attachObject( manual_object );

      manual_objects_[ i ] = manual_object;
    }
    break;

  case BILLBOARDS: // billboards with configurable width
    billboard_lines_.resize( buffer_length );
    for( size_t i = 0; i < billboard_lines_.size(); i++ )
    {
      rviz::BillboardLine* billboard_line = new rviz::BillboardLine(scene_manager_, scene_node_);
      billboard_lines_[ i ] = billboard_line;
    }
    break;
  }


}

bool validateFloats( const nav_msgs::Odometry& msg )
{
    bool valid = true;
    valid = valid && rviz::validateFloats( msg.pose.pose );
    valid = valid && rviz::validateFloats( msg.twist.twist );
    return valid;
}

//void OdometryToPathDisplay::processMessage( const nav_msgs::Path::ConstPtr& msg )
void OdometryToPathDisplay::processMessage( const nav_msgs::Odometry::ConstPtr& msg )
{



  // Calculate index of oldest element in cyclic buffer
  size_t bufferIndex = messages_received_ % buffer_length_property_->getInt();

  LineStyle style = (LineStyle) style_property_->getOptionInt();
  Ogre::ManualObject* manual_object = NULL;
  rviz::BillboardLine* billboard_line = NULL;

  // Delete oldest element
  switch(style)
  {
  case LINES:
    manual_object = manual_objects_[ bufferIndex ];
    manual_object->clear();
    break;

  case BILLBOARDS:
    billboard_line = billboard_lines_[ bufferIndex ];
    billboard_line->clear();
    break;
  }

  // Check if path contains invalid coordinate values
  if( !validateFloats( *msg ))
  {
    setStatus( rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  // Lookup transform into fixed frame
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->getTransform( msg->header, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  Ogre::Matrix4 transform( orientation );
  transform.setTrans( position );

//  scene_node_->setPosition( position );
//  scene_node_->setOrientation( orientation );

  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  //uint32_t num_points = msg->poses.size();
  float line_width = line_width_property_->getFloat();

  if(first_time_==true)
  {
      last_pos_ = msg->pose.pose.position;
      first_time_=false;
      return;
  }

  switch(style)
  {
  case LINES:
  {

    manual_object->estimateVertexCount( 3 );//num_points );
    manual_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );

    Ogre::Vector3 xpos_aux = transform * Ogre::Vector3( last_pos_.x, last_pos_.y, last_pos_.z );
    manual_object->position( xpos_aux.x, xpos_aux.y, xpos_aux.z );
    manual_object->colour( color );


    const geometry_msgs::Point& pos = msg->pose.pose.position;
    last_pos_ = msg->pose.pose.position;
    Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z );
    manual_object->position( xpos.x, xpos.y, xpos.z );
    manual_object->colour( color );


    manual_object->end();
  }
    break;

  case BILLBOARDS:
  {

    billboard_line->setNumLines( 1 );
    billboard_line->setMaxPointsPerLine( 2 );
    billboard_line->setLineWidth( line_width );

    Ogre::Vector3 xpos_aux = transform * Ogre::Vector3( last_pos_.x, last_pos_.y, last_pos_.z );
    billboard_line->addPoint( xpos_aux, color );

    const geometry_msgs::Point& pos = msg->pose.pose.position;
    last_pos_ = msg->pose.pose.position;
    Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z );
    billboard_line->addPoint(xpos, color);

  }
    break;
  }

}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_plugin_odometry_to_path::OdometryToPathDisplay, rviz::Display )
