// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <std_msgs/msg/int32.hpp>

#include "geometry.h"


using std::placeholders::_1;

class BasicControlsNode : public rclcpp::Node
{
public:
  explicit BasicControlsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~BasicControlsNode() = default;

  inline void
  applyChanges()
  {
    server_->applyChanges();
  }

  visualization_msgs::msg::Marker makeCylinder(const visualization_msgs::msg::InteractiveMarker & msg, vector_t & scale);

  void addInteractiveMarker(vector_t & marker_pos, quat_t & marker_quat, vector_t & marker_scale, std::string & marker_name);

  void removeInteractiveMarker(std::string & marker_name);

  void flipMarkerState();

  void regulateMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr msg);


private:

  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr flip_timer_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr pl_marker_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr selected_id_pub_;

  std::vector<int> current_pl_ids_;
};  

BasicControlsNode::BasicControlsNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("basic_controls", options)
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "basic_controls",
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_topics_interface(),
    get_node_services_interface());

   // create a timer to flip marker state periodically
  // flip_timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&BasicControlsNode::flipMarkerState, this));

  pl_marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
			"/pl_marker_array",	10,
			std::bind(&BasicControlsNode::regulateMarkers, this, std::placeholders::_1));

  selected_id_pub_ = this->create_publisher<std_msgs::msg::Int32>("/selected_pl_id", 10);
}


visualization_msgs::msg::Marker BasicControlsNode::makeCylinder(const visualization_msgs::msg::InteractiveMarker & msg, vector_t & scale)
{
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.scale.x = msg.scale * scale(0);
  marker.scale.y = msg.scale * scale(1);
  marker.scale.z = msg.scale * scale(2);
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.6;

  return marker;
}


void BasicControlsNode::regulateMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{

// remove markers that have disappeared
// REWRITE - remove gone markers, only add new ones
  for (size_t i = 0; i < this->current_pl_ids_.size(); i++)
  {
    for (size_t j = 0; j < msg->markers.size(); j++)
    {
      if (current_pl_ids_.at(i) == msg->markers.at(j).id)
      {
        break;
      }
      if (j == msg->markers.size()-1)
      {
        std::string marker_name = std::to_string(i);
        this->removeInteractiveMarker(marker_name);
      }
     
    }
    
  }
  

  for (size_t i = 0; i < msg->markers.size(); i++)
  {

    vector_t pos;
    pos(0) = msg->markers.at(i).pose.position.x;
    pos(1) = msg->markers.at(i).pose.position.y;
    pos(2) = msg->markers.at(i).pose.position.z;

    quat_t quat;
    quat(0) = msg->markers.at(i).pose.orientation.w;
    quat(1) = msg->markers.at(i).pose.orientation.x;
    quat(2) = msg->markers.at(i).pose.orientation.y;
    quat(3) = msg->markers.at(i).pose.orientation.z;
    
    vector_t scale;
    scale(0) = msg->markers.at(i).scale.x;
    scale(1) = msg->markers.at(i).scale.y;
    scale(2) = msg->markers.at(i).scale.z;

    std::string name = std::to_string(msg->markers.at(i).id);

    BasicControlsNode::addInteractiveMarker(pos, quat, scale, name);

    current_pl_ids_.push_back(msg->markers.at(i).id);

  }
}


void BasicControlsNode::flipMarkerState() 
// add service to add and remove marker from another node
// listen to marker topic, hijack and make an interactive marker
// use marker array in publisher for easy synchronization?
{
  static bool marker_state = false;

  if (marker_state)
  {
    vector_t marker_pos;
    marker_pos(0) = 0.0;
    marker_pos(1) = 0.0;
    marker_pos(2) = 3.0;

    orientation_t marker_rpy; // not sure why this order, probably marker definition
    marker_rpy(2) = 0;
    marker_rpy(0) = 1.57;
    marker_rpy(1) = 1.57;

    quat_t marker_quat = eulToQuat(marker_rpy);

    vector_t marker_scale;
    marker_scale(0) = 0.5;
    marker_scale(1) = 0.5;
    marker_scale(2) = 5.0;

    std::string marker_name = "test_cylinder";

    this->addInteractiveMarker(marker_pos, marker_quat, marker_scale, marker_name);
  }
  else
  {
    std::string marker_name = "test_cylinder";
    this->removeInteractiveMarker(marker_name);
  }
  
  marker_state = not marker_state;
}


void BasicControlsNode::removeInteractiveMarker(std::string & marker_name)
{
  this->server_->erase(marker_name);

  this->applyChanges();
}


void BasicControlsNode::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  // std::ostringstream oss;
  // oss << "Feedback from marker '" << feedback->marker_name << "' " <<
  //   " / control '" << feedback->control_name << "'";

  // std::ostringstream mouse_point_ss;
  // if (feedback->mouse_point_valid) {
  //   mouse_point_ss << " at " << feedback->mouse_point.x <<
  //     ", " << feedback->mouse_point.y <<
  //     ", " << feedback->mouse_point.z <<
  //     " in frame " << feedback->header.frame_id;
  // }

  static double click_start_x;
  static double click_start_y;

  static double click_end_x;
  static double click_end_y;

  switch (feedback->event_type) {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
      // oss << ": button click" << mouse_point_ss.str() << ".";
      // RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN:
      // oss << ": mouse down" << mouse_point_ss.str() << ".";
      // RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      click_start_x = feedback->mouse_point.x;
      click_start_y = feedback->mouse_point.y;
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
      // oss << ": mouse up" << mouse_point_ss.str() << ".";
      // RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      click_end_x = feedback->mouse_point.x;
      click_end_y = feedback->mouse_point.y;

      if (click_start_x == click_end_x && click_start_y == click_end_y)
      {
        auto int_msg = std_msgs::msg::Int32();
        int_msg.data = std::stoi(feedback->marker_name);
        selected_id_pub_->publish(int_msg);
        RCLCPP_INFO(get_logger(), "Clicked marker number %d", std::stoi(feedback->marker_name));
      }

      break;

  }

  server_->applyChanges();
}



void BasicControlsNode::addInteractiveMarker(vector_t & marker_pos, quat_t & marker_quat, vector_t & marker_scale, std::string & marker_name)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  int_marker.pose.position.x = marker_pos(0);
  int_marker.pose.position.y = marker_pos(1);
  int_marker.pose.position.z = marker_pos(2);
  int_marker.pose.orientation.w = marker_quat(0);
  int_marker.pose.orientation.x = marker_quat(1);
  int_marker.pose.orientation.y = marker_quat(2);
  int_marker.pose.orientation.z = marker_quat(3);
  int_marker.scale = 1;

  int_marker.name = marker_name;
  int_marker.description = marker_name;

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  visualization_msgs::msg::Marker marker = makeCylinder(int_marker, marker_scale);
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processFeedback, this, _1));

  this->applyChanges();
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto basic_controls = std::make_shared<BasicControlsNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(basic_controls);

  RCLCPP_INFO(basic_controls->get_logger(), "Ready");
  executor.spin();
  rclcpp::shutdown();

  return 0;
}






