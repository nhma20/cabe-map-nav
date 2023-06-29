/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <std_msgs/msg/int32.hpp>

#include <nav_msgs/msg/path.hpp>

#include <radar_cable_follower_msgs/msg/tracked_powerlines.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>

#include <stdint.h>
#include <stdlib.h> 
#include <math.h>  
#include <limits>
#include <mutex>
#include <chrono>
#include <iostream>

#include "geometry.h"

#include "../include/AStar.hpp"


#define PI 3.14159265
#define NAN_ std::numeric_limits<double>::quiet_NaN()

using namespace std::chrono;
using namespace std::chrono_literals;
// using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
		_offboard_control_mode_publisher =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		_trajectory_setpoint_publisher =
			this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		_vehicle_command_publisher =
			this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/vehicle_command/in", 10);

		this->declare_parameter<float>("yaw_frac", 0.25);
		this->declare_parameter<float>("pos_frac", 0.5);
		this->declare_parameter<float>("powerline_following_distance", 5.0);
		this->get_parameter("powerline_following_distance", _following_distance);
		this->declare_parameter<float>("powerline_following_speed", 1.5);
		this->get_parameter("powerline_following_speed", _follow_speed);
		this->declare_parameter<int>("powerline_following_ID", -1);

		this->declare_parameter<int>("launch_with_debug", 1);
		this->get_parameter("launch_with_debug", _launch_with_debug);

		this->declare_parameter<float>("take_off_to_height", 0.0);
		this->get_parameter("take_off_to_height", _takeoff_height);

		this->declare_parameter<float>("astar_map_scale", 4.0);
		this->get_parameter("astar_map_scale", _astar_map_scale);

		this->declare_parameter<float>("astar_safe_zone_radius", 1.0);
		this->get_parameter("astar_safe_zone_radius", _astar_safe_zone_radius);

		this->declare_parameter<float>("astar_map_buffer", 3.0);
		this->get_parameter("astar_map_buffer", _astar_map_buffer);


		// VehicleStatus: https://github.com/PX4/px4_msgs/blob/master/msg/VehicleStatus.msg
		_vehicle_status_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/vehicle_status/out", 10,
            [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
              _arming_state = msg->arming_state;
              _nav_state = msg->nav_state;
			});


		_rc_channels_sub = this->create_subscription<px4_msgs::msg::RcChannels>(
			"/fmu/rc_channels/out",	10,
            [this](px4_msgs::msg::RcChannels::ConstSharedPtr msg) {
              _rc_misc_state = msg->channels[7];
			  _rc_height_state = msg->channels[6];
			  
			//   RCLCPP_INFO(this->get_logger(),  "\nRC MISC state: %f", _rc_misc_state);
			});


		_powerline_pose_sub = this->create_subscription<radar_cable_follower_msgs::msg::TrackedPowerlines>(
			"/tracked_powerlines",	10,
			std::bind(&OffboardControl::update_alignment_pose, this, std::placeholders::_1));


		_selected_id_sub = this->create_subscription<std_msgs::msg::Int32>(
			"/selected_pl_id",	10,
            [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
				_selected_pl_id = msg->data;
			});


		_start_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/plane_start_point", 10);


		_global_position_sub = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
		"/fmu/vehicle_global_position/out",	10,
		[this](px4_msgs::msg::VehicleGlobalPosition::ConstSharedPtr msg) {
			_global_latitude = (float)msg->lat; // degrees
			_global_longitude = (float)msg->lon; // degrees
		}); 


		_plane_line_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		"/plane_lines_pcl",	10,
		[this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
			_plane_line_points_msg = msg;
		}); 


		_plane_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/plane_pose",	10,
		[this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
			_plane_pose_msg = msg;
		}); 


		_transformed_plane_pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/trans_plane_pcl", 10);


		_plane_path_pub = this->create_publisher<nav_msgs::msg::Path>("/plane_path", 10);



		if(_launch_with_debug > 0)
		{
			_follow_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/follow_pose", 10);
			_manual_path_pub = this->create_publisher<nav_msgs::msg::Path>("/manual_path", 10);
			_offboard_path_pub = this->create_publisher<nav_msgs::msg::Path>("/offboard_path", 10);
		}

		tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
		transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


		// get common timestamp
		_timesync_sub =	this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					_timestamp.store(msg->timestamp);
				});


		timer_ = this->create_wall_timer(100ms, 
				std::bind(&OffboardControl::flight_state_machine, this));

		_mission_timer = this->create_wall_timer(250ms, 
				std::bind(&OffboardControl::mission_state_machine, this));

		_path_timer = this->create_wall_timer(500ms, 
				std::bind(&OffboardControl::publish_path, this));
		
	}


	~OffboardControl() {
		RCLCPP_INFO(this->get_logger(),  "Shutting down offboard control, landing..");
		publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); 
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		
	}

	void arm() const;
	void disarm() const;
	


private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr _mission_timer;
	rclcpp::TimerBase::SharedPtr _path_timer;

	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_control_mode_publisher;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_publisher;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_publisher;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _follow_pose_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _manual_path_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _offboard_path_pub;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _start_point_pub;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _transformed_plane_pcl_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _plane_path_pub;

	rclcpp::Subscription<radar_cable_follower_msgs::msg::TrackedPowerlines>::SharedPtr _powerline_pose_sub;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _selected_id_sub;
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr _rc_channels_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr _global_position_sub;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _plane_line_points_sub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _plane_pose_sub;

	std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::atomic<uint64_t> _timestamp;   //!< common synced timestamped
	int _nav_state, _old_nav_state = 0;
	int _arming_state;
	geometry_msgs::msg::PoseArray::SharedPtr _powerline_array_msg; // auto?
	int _counter = 0;
	float _following_distance;
	float _follow_speed;
	int _selected_ID = -1;
	int _launch_with_debug;
	float _takeoff_height;

	bool _new_takeoff = true;
	float _rc_misc_state = -1;
	float _prev_rc_misc_state = -2;

	float _rc_height_state = -1;
	float _prev_rc_height_state = -1;

    bool _printed_offboard = false;

	bool _in_offboard = false;

	std::mutex _id_mutex;
	std::mutex _drone_pose_mutex;
	std::mutex _powerline_mutex;

	pose_t _drone_pose; // in world coordinates North-West-Up
	pose_t _alignment_pose;

	float _hover_height = 2;

	float _global_latitude = 0.0; // degrees
	float _global_longitude = 0.0; // degrees

	int _selected_pl_id = -1;

	float _astar_map_buffer = 0.0;
	float _astar_safe_zone_radius = 0.0;
	float _astar_map_scale = 0.0;

	sensor_msgs::msg::PointCloud2::SharedPtr _plane_line_points_msg;
	geometry_msgs::msg::PoseStamped::SharedPtr _plane_pose_msg;

	void publish_path();
	void mission_state_machine();
	void flight_state_machine();
	void update_drone_pose();
	void update_alignment_pose(radar_cable_follower_msgs::msg::TrackedPowerlines::SharedPtr msg);
	void publish_offboard_control_mode() const;
	void publish_takeoff_setpoint();
	void publish_tracking_setpoint();
	void publish_hold_setpoint() const;
	void publish_setpoint(px4_msgs::msg::TrajectorySetpoint msg) const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
	void world_to_points();
	void latlon_to_xy(double lat1, double lon1, double lat2, double lon2, float & x_out, float & y_out);
	void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, sensor_msgs::msg::PointCloud2 * pcl_msg);
	void pathplanner(point2d_t point_start, point2d_t point_goal, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_plane_points,
										pcl::PointCloud<pcl::PointXYZ>::Ptr path_pcl);
};


void OffboardControl::publish_path() {
	// limit length
	// offboard and manual paths

	static auto manual_path_msg = nav_msgs::msg::Path();
	manual_path_msg.header.stamp = this->now();
	manual_path_msg.header.frame_id = "world";

	static auto offboard_path_msg = nav_msgs::msg::Path();
	offboard_path_msg.header.stamp = this->now();
	offboard_path_msg.header.frame_id = "world";

	auto pose_msg = geometry_msgs::msg::PoseStamped();
	pose_msg.header.stamp = this->now();
	pose_msg.header.frame_id = "world";

	_drone_pose_mutex.lock(); {

		pose_msg.pose.position.x = _drone_pose.position(0);
		pose_msg.pose.position.y = _drone_pose.position(1);
		pose_msg.pose.position.z = _drone_pose.position(2);

		pose_msg.pose.orientation.x = _drone_pose.quaternion(0);
		pose_msg.pose.orientation.y = _drone_pose.quaternion(1);
		pose_msg.pose.orientation.z = _drone_pose.quaternion(2);
		pose_msg.pose.orientation.w = _drone_pose.quaternion(3);

	} _drone_pose_mutex.unlock();

	static float path_segment_length = 0.5; // meters

	if (_in_offboard)
	{
		float dist = 99999.9;

		if (offboard_path_msg.poses.size() > 0 )
		{		
			float x_diff = offboard_path_msg.poses.back().pose.position.x - pose_msg.pose.position.x;
			float y_diff = offboard_path_msg.poses.back().pose.position.y - pose_msg.pose.position.y;
			float z_diff = offboard_path_msg.poses.back().pose.position.z - pose_msg.pose.position.z;
			dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2));
		}		

		if(dist > path_segment_length)
		{
			offboard_path_msg.poses.push_back(pose_msg);
			_offboard_path_pub->publish(offboard_path_msg);
		}
		manual_path_msg.poses.clear();
	}
	else
	{
		float dist = 99999.9;

		if (manual_path_msg.poses.size() > 0 )
		{	
			float x_diff = manual_path_msg.poses.back().pose.position.x - pose_msg.pose.position.x;
			float y_diff = manual_path_msg.poses.back().pose.position.y - pose_msg.pose.position.y;
			float z_diff = manual_path_msg.poses.back().pose.position.z - pose_msg.pose.position.z;
			dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2));
		}	

		if(dist > path_segment_length)
		{
			manual_path_msg.poses.push_back(pose_msg);
			_manual_path_pub->publish(manual_path_msg);
		}
		offboard_path_msg.poses.clear();
	}
		
}


void OffboardControl::mission_state_machine() {

	//if (! _in_offboard)
	//{
	//	return;
	//}

	// if (_rc_misc_state < -0.5)
	// {
	// 	RCLCPP_INFO(this->get_logger(),  "\nOriginal distance and speed\n");
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_distance", _following_distance));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_speed", _follow_speed));
	// }

	// if (_prev_rc_misc_state < -0.5 && _rc_misc_state > -0.5 && _rc_misc_state < 0.5)
	// {
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_distance", _following_distance+7.5));
	// 	RCLCPP_INFO(this->get_logger(),  "\nIncreasing following distance\n");
	// }

	// if (_prev_rc_misc_state > -0.5 && _prev_rc_misc_state < 0.5 && _rc_misc_state > 0.5)
	// {
	// 	// this->set_parameter(rclcpp::Parameter("powerline_following_ID", 0));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_speed", _follow_speed*2));
	// 	RCLCPP_INFO(this->get_logger(),  "\nIncreasing following speed\n");
	// }

	if (_prev_rc_misc_state != _rc_misc_state)
	{
	
		if (_rc_misc_state < -0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\nForward direction\n");
			_follow_speed = abs(_follow_speed);
		}

		if (_rc_misc_state > -0.5 && _rc_misc_state < 0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\nStop\n");
			_follow_speed = 0.0;
		}

		if (_rc_misc_state > 0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\nReverse direction\n");
			_follow_speed = -abs(_follow_speed);
		}

		this->set_parameter(rclcpp::Parameter("powerline_following_speed", _follow_speed));

		_prev_rc_misc_state = _rc_misc_state;
	}
	

	if (_prev_rc_height_state != _rc_height_state)
	{

		if (_rc_height_state < -0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\n-1m following height\n");
			_following_distance = _following_distance - 1.0;
		}

		if (_rc_height_state > -0.5 && _rc_height_state < 0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\nSame following height\n");
		}

		if (_rc_height_state > 0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\n+1m following height\n");
			_following_distance = _following_distance + 1.0;
		}

		this->set_parameter(rclcpp::Parameter("powerline_following_distance", _following_distance));

		_prev_rc_height_state = _rc_height_state;
	}

	// static int callback_count = 0;

	// if (callback_count == 0)
	// {
	// 	RCLCPP_INFO(this->get_logger(),  "\nOriginal ID, original direction\n");
	// }

	// if (callback_count == 1)
	// {
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_ID", 0));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_distance", _following_distance));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_speed", -_follow_speed*2));
	// 	RCLCPP_INFO(this->get_logger(),  "\nNew ID, reversing direction\n");
	// }

	// if (callback_count == 2)
	// {
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_ID", 0));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_distance", _following_distance+7.5));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_speed", _follow_speed*2));
	// 	RCLCPP_INFO(this->get_logger(),  "\nGreater distance, increased speed\n");
	// }
	

	// callback_count++;
}


void OffboardControl::latlon_to_xy(double lat1, double lon1, double lat2, double lon2, float &x_out, float &y_out) {

	double p = (double)PI/180.0;
    double a = 0.5 - cos((lat2-lat1)*p)/2.0; 
	double b = cos(lat1*p) * cos(lat2*p) * (1-cos((lon2-lon1)*p))/2.0;
    double distance = 12742.0 * asin(sqrt(a+b)) * 1000.0; // 2*R*asin... to meters

    double y = sin(p*(lon2-lon1)) * cos(p*lat2);
    double x = cos(p*lat1)*sin(p*lat2) - sin(p*lat1)*cos(p*lat2)*cos(p*(lon2-lon1));
    double bearing = atan2(y,x);
    bearing = bearing / p;
    bearing = fmod( ( bearing + 360.0), 360);

    x_out = (float)(cos(p*bearing) * distance);
    y_out = (float)(sin(p*bearing) * distance);

}


void OffboardControl::create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, sensor_msgs::msg::PointCloud2 * pcl_msg) {

  // create PointCloud2 msg
	//https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/tests/system.cpp
	int pcl_size = cloud->size();
	auto pcl2_msg = sensor_msgs::msg::PointCloud2();
	const uint32_t POINT_STEP = 12;

	pcl_msg->header = std_msgs::msg::Header();
	pcl_msg->header.stamp = this->now();
	std::string frameID = "world";
	pcl_msg->header.frame_id = frameID;
	pcl_msg->fields.resize(3);
	pcl_msg->fields[0].name = 'x';
	pcl_msg->fields[0].offset = 0;
	pcl_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[0].count = 1;
	pcl_msg->fields[1].name = 'y';
	pcl_msg->fields[1].offset = 4;
	pcl_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[1].count = 1;
	pcl_msg->fields[2].name = 'z';
	pcl_msg->fields[2].offset = 8;
	pcl_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[2].count = 1;

	if(pcl_size > 0){
		pcl_msg->data.resize(std::max((size_t)1, (size_t)pcl_size) * POINT_STEP, 0x00);
	} else {
        return;
    }

	pcl_msg->point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
	pcl_msg->row_step = pcl_msg->data.size();//pcl_msg->point_step * pcl_msg->width; // only 1 row because unordered
	pcl_msg->height = 1;  // because unordered cloud
	pcl_msg->width = pcl_msg->row_step / POINT_STEP; // number of points in cloud
	pcl_msg->is_dense = false; // there may be invalid points

	// fill PointCloud2 msg data
	uint8_t *ptr = pcl_msg->data.data();

	for (size_t i = 0; i < (size_t)pcl_size; i++)
	{
		pcl::PointXYZ point = (*cloud)[i];

        *(reinterpret_cast<float*>(ptr + 0)) = point.x;
        *(reinterpret_cast<float*>(ptr + 4)) = point.y;
        *(reinterpret_cast<float*>(ptr + 8)) = point.z;
        ptr += POINT_STEP;
	}
	
}


void OffboardControl::read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// read PointCloud2 msg data
	int pcl_size = msg->width;
	uint8_t *ptr = msg->data.data();
	const uint32_t POINT_STEP = 12;


	for (size_t i = 0; i < (size_t)pcl_size; i++) 
	{
		pcl::PointXYZ point(
				(float)(*(reinterpret_cast<float*>(ptr + 0))),
				(float)(*(reinterpret_cast<float*>(ptr + 4))),
				(float)(*(reinterpret_cast<float*>(ptr + 8)))
			);

		cloud->push_back(point);	

		ptr += POINT_STEP;
	}
}   


void OffboardControl::pathplanner(point2d_t point_start, point2d_t point_goal, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_plane_points,
										pcl::PointCloud<pcl::PointXYZ>::Ptr path_pcl) {


	auto start_1 = high_resolution_clock::now();

    // find highest X and Y values
    float highest_x = 0;
    float highest_y = 0;
    float smallest_x = 999999;
    float smallest_y = 999999;

    for (int i = 0; i < (int)transformed_plane_points->size(); i++)
    {
        if (transformed_plane_points->points[i].y > highest_x)
        {
            highest_x = transformed_plane_points->points[i].y;
        }

        if (transformed_plane_points->points[i].z > highest_y)
        {
            highest_y = transformed_plane_points->points[i].z;
        }

        if (transformed_plane_points->points[i].y < smallest_x)
        {
            smallest_x = transformed_plane_points->points[i].y;
        }

        if (transformed_plane_points->points[i].z < smallest_y)
        {
            smallest_y = transformed_plane_points->points[i].z;
        }
    }


	if (point_start(0) > highest_x)
	{
		highest_x = point_start(0);
	}

	if (point_start(1) > highest_y)
	{
		highest_y = point_start(1);
	}

	if (point_start(0) < smallest_x)
	{
		smallest_x = point_start(0);
	}

	if (point_start(1) < smallest_y)
	{
		smallest_y = point_start(1);
	}


		if (point_goal(0) > highest_x)
	{
		highest_x = point_goal(0);
	}

	if (point_goal(1) > highest_y)
	{
		highest_y = point_goal(1);
	}

	if (point_goal(0) < smallest_x)
	{
		smallest_x = point_goal(0);
	}

	if (point_goal(1) < smallest_y)
	{
		smallest_y = point_goal(1);
	}

    // Calculate map size in X and Y, scale to some resolution
    // float scale = 4;
	float scale = _astar_map_scale;
    int size_x = (int)(highest_x*scale) - (int)(smallest_x*scale);
    int size_y = (int)(highest_y*scale) - (int)(smallest_y*scale);

	RCLCPP_INFO(this->get_logger(), "Path map size: X %d, Y %d", size_x, size_y);

    // Add free-space buffer on each side of map and on top
    // int buffer_size = 3; // 3m buffer 
	int buffer_size = (int)_astar_map_buffer; // 3m buffer 
    buffer_size = (int)(buffer_size * scale);
    size_x = size_x + 2*(int)(buffer_size);
    size_y = size_y + 2*(int)(buffer_size);

    // Create A* map, sized based on powerline coordinates (10cm resolution)
    AStar::Generator generator;
    generator.setWorldSize({size_x, size_y});

    // Add collisions based on powerline coordinates
    std::vector<std::vector<int>> scaled_collision_coordinates;
    // float min_powerline_safety_distance = 1.0; // meters
	float min_powerline_safety_distance = _astar_safe_zone_radius;
    int scaled_safety_distance = (int)ceil(min_powerline_safety_distance * scale);

    // initialize collision coordinates vector with scaled powerline coordinates
    for (int i = 0; i < (int)transformed_plane_points->size(); i++)
    {
        std::vector<int> tmp_vec;
        tmp_vec.push_back((int)(transformed_plane_points->points[i].y*scale + buffer_size));
        tmp_vec.push_back((int)(transformed_plane_points->points[i].z*scale));//+buffer_size);
        scaled_collision_coordinates.push_back(tmp_vec);
    }


	auto stop_1 = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(stop_1 - start_1);
	RCLCPP_INFO(this->get_logger(), "Pathplanning step 1 took: %f ms", (((float)duration.count()) / 1000));

	auto start_2 = high_resolution_clock::now();

    int collisions_count = 0;

    // Make collision zones around plane powerline points
    for (int i  = 0; i < size_y; i++)
    {
        for (int j = 0; j < size_x; j++)
        {
            for (int k = 0; k < (int)scaled_collision_coordinates.size(); k++)
            {
                // calculate distance from each map point to each line coordinate
                float dist_x = scaled_collision_coordinates.at(k).at(0) - j;
                float dist_y = scaled_collision_coordinates.at(k).at(1) - i;
                float euclid_dist = ceil(sqrt( (float)pow(dist_x, 2) + (float)pow(dist_y, 2) ));

                // if a distance is below the safety distance, make collision
                if (abs(euclid_dist) <= scaled_safety_distance)
                {
                    generator.addCollision({j, i});
					collisions_count++;
                }
            }            
            
        }
        
    }

	auto stop_2 = high_resolution_clock::now();
	auto duration2 = duration_cast<microseconds>(stop_2 - start_2);
	RCLCPP_INFO(this->get_logger(), "Pathplanning step 2 took: %f ms", (((float)duration2.count()) / 1000));

	RCLCPP_INFO(this->get_logger(), "Pathplanner number of collisions: %d", collisions_count);
    

	auto start_3 = high_resolution_clock::now();

    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

	// scale and add buffer to start and goal points
	point2d_t scaled_point_start;
	scaled_point_start(0) = (point_start(0)*scale + buffer_size);
	scaled_point_start(1) = (point_start(1)*scale);

	// scale and add buffer to start and goal points
	point2d_t scaled_point_goal;
	scaled_point_goal(0) = (point_goal(0)*scale + buffer_size);
	scaled_point_goal(1) = (point_goal(1)*scale);

	RCLCPP_INFO(this->get_logger(), "Generating path ...");

	auto start4 = high_resolution_clock::now();

    auto path = generator.findPath({(int)scaled_point_start(0), (int)scaled_point_start(1)}, {(int)scaled_point_goal(0), (int)scaled_point_goal(1)});

	auto stop4 = high_resolution_clock::now();
	float duration4 = (float)duration_cast<microseconds>(stop4 - start4).count() / 1000;

	RCLCPP_INFO(this->get_logger(), "Generating path took %f ms", duration4);

    for(auto& coordinate : path) {
		// RCLCPP_INFO(this->get_logger(), "%d, %d", coordinate.x, coordinate.y);

		pcl::PointXYZ tmp_point;
		tmp_point.x = 0.0;
		tmp_point.y = (float)(coordinate.x - buffer_size) / (float)scale;
		tmp_point.z = (float)coordinate.y / (float)scale;

		path_pcl->push_back(tmp_point);
    }


	auto stop_3 = high_resolution_clock::now();
	auto duration3 = duration_cast<microseconds>(stop_3 - start_3);
	RCLCPP_INFO(this->get_logger(), "Pathplanning step 3 took: %f ms", (((float)duration3.count()) / 1000));

	// std::string astar_map = generator.visualize(path); // send to file instead?
	
	// // RCLCPP_INFO(this->get_logger(), "Generated map: \n%s", astar_map);

	// // Create and open a text file
	// std::ofstream MyFile("astar_map.txt");

	// // Write to the file
	// MyFile << astar_map;

	// // Close the file
	// MyFile.close();

}


void OffboardControl::flight_state_machine() {

	OffboardControl::update_drone_pose();

	// If drone not armed (from external controller) and put in offboard mode, do nothing
	if (_nav_state != 14) 
	{
		if (_old_nav_state != _nav_state && _nav_state != 14 && _launch_with_debug > 0)
		{				
			RCLCPP_INFO(this->get_logger(), "nav_state: %d", _nav_state);
			RCLCPP_INFO(this->get_logger(), "\n \nWaiting for offboard mode\n");
		}

		if (_old_nav_state != _nav_state && _nav_state == 14 && _launch_with_debug > 0)
		{				
			RCLCPP_INFO(this->get_logger(), "nav_state: %d", _nav_state);
			RCLCPP_INFO(this->get_logger(), "\n \nOffboard mode enabled\n");
		}

		publish_hold_setpoint();
		_in_offboard = false;
		_new_takeoff = true;
		_old_nav_state = _nav_state;
		_counter = 0;
		return;
	}

	_in_offboard = true;

	if (!_printed_offboard)
	{
		RCLCPP_INFO(this->get_logger(), "\n \nEntering offboard control \n");
		_printed_offboard = true;
		this->arm();
	}



	/* ----- perform mapping based on approximate GPS waypoints ----- */

	static float takeoff_yaw = 9999.9;

	if (takeoff_yaw > 9999.0) {

		takeoff_yaw = -quatToEul(_drone_pose.quaternion)(2);

	}
		

	// lat, long, z, yaw
	static double map_pos_array[5][4] = {{_global_latitude, _global_longitude, 2.5, takeoff_yaw}, // takeoff
										{55.47029419, 10.32891724, 2.5, -1.195}, // north of middle of span, 2.5m
										{55.47029419, 10.32891724, 13.5, -1.195}, // north of middle of span, 13.5m
										{55.47018588, 10.32896105, 13.5, -1.195}, // middle of span, 13.5m
										{55.47012786, 10.32933877, 13.5, -1.195}};//, // beyond east tower, 13.5m
										//{55.47023526, 10.32855140, 13.5}}; // beyond west tower, 13.5m

	static int map_pos_cnt = 0;

	if (map_pos_cnt < (int)(sizeof(map_pos_array)/sizeof(map_pos_array[0]))) { //6

		// subscribe to vehicle_global_position
		// use latlon_to_xy() to find XY offset between target latlong (from array) and current latlong
		// translate local XY offset into world frame (waypoint_xy + world_xy)?

		float gps_local_x = 0.0;
		float gps_local_y = 0.0;
		OffboardControl::latlon_to_xy(_global_latitude, _global_longitude, 
										map_pos_array[map_pos_cnt][0], map_pos_array[map_pos_cnt][1], 
										gps_local_x, gps_local_y);

		// relative XY to world XY (incl. ENU to NED)
		float gps_waypoint_world_coordinate_x = _drone_pose.position(0) + gps_local_x;
		float gps_waypoint_world_coordinate_y = -1*_drone_pose.position(1) + gps_local_y;
		float gps_waypoint_world_coordinate_z = -1*map_pos_array[map_pos_cnt][2];
		float gps_waypoint_world_yaw = map_pos_array[map_pos_cnt][3];

		px4_msgs::msg::TrajectorySetpoint msg{};
		msg.timestamp = _timestamp.load();
		msg.x = gps_waypoint_world_coordinate_x; // in meters NED
		msg.y = gps_waypoint_world_coordinate_y; // in meters NED
		msg.z = gps_waypoint_world_coordinate_z; // in meters NED
		msg.yaw = gps_waypoint_world_yaw; // rotation around z NED in radians
		msg.vx = 0.0; // m/s NED
		msg.vy = 0.0; // m/s NED
		msg.vz = 0.0; // m/s NED

		OffboardControl::publish_setpoint(msg);

		// XYZ delta to goal waypoint (incl. ENU to NED)
		float x_diff = _drone_pose.position(0) - gps_waypoint_world_coordinate_x;
		float y_diff = -1*_drone_pose.position(1) - gps_waypoint_world_coordinate_y;
		float z_diff = -1*_drone_pose.position(2) - gps_waypoint_world_coordinate_z;

		float threshold = 0.25;
		if ( x_diff < threshold &&
			 y_diff < threshold &&
			 z_diff < threshold &&
			 x_diff > -threshold &&
			 y_diff > -threshold &&
			 z_diff > -threshold
		)
		{
			map_pos_cnt++;
		}
		else {
			RCLCPP_INFO(this->get_logger(), "\n Goal delta to position %d: \n X: %f \n Y: %f \n Z: %f \n", map_pos_cnt, x_diff, y_diff, z_diff);
		}

		return;

	}


	// create plane

	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_line_points (new pcl::PointCloud<pcl::PointXYZ>);
	
	OffboardControl::read_pointcloud(_plane_line_points_msg, plane_line_points);

	float lowest_powerline_z = 99999.0;
	for (int i = 0; i < (int)plane_line_points->size(); i++)
	{
		if (plane_line_points->points[i].z < lowest_powerline_z)
		{
			lowest_powerline_z = plane_line_points->points[i].z;
		}
		
	}

	homog_transform_t plane_to_world;

	point_t plane_position;
	plane_position(0) = _plane_pose_msg->pose.position.x;
	plane_position(1) = _plane_pose_msg->pose.position.y;
	plane_position(2) = _plane_pose_msg->pose.position.z;

	quat_t plane_quaternion;
	plane_quaternion(0) = _plane_pose_msg->pose.orientation.x;
	plane_quaternion(1) = _plane_pose_msg->pose.orientation.y;
	plane_quaternion(2) = _plane_pose_msg->pose.orientation.z;
	plane_quaternion(3) = _plane_pose_msg->pose.orientation.w;

	
	homog_point_t drone_start_position_in_plane;
	drone_start_position_in_plane(0) = plane_position(0);
	drone_start_position_in_plane(1) = plane_position(1);
	drone_start_position_in_plane(2) = plane_position(2) + _following_distance;
	drone_start_position_in_plane(3) = 1;


	//visualize drone start point in plane
	geometry_msgs::msg::PointStamped point_msg;
	point_msg.header.frame_id = "world";
	point_msg.header.stamp = this->get_clock()->now();
	point_msg.point.x = plane_position(0);
	point_msg.point.y = plane_position(1);
	point_msg.point.z = plane_position(2) + _following_distance;
	_start_point_pub->publish(point_msg);


	static bool planned_path = false;
	static int selected_pl_id_old = -1;
	static int path_pos_cnt = 0;
	static pcl::PointCloud<pcl::PointXYZ>::Ptr planned_path_pcl (new pcl::PointCloud<pcl::PointXYZ>);

	if (_selected_pl_id < 0)
	{
		// hold position until powerline has been selected for alignment

		px4_msgs::msg::TrajectorySetpoint msg{};
		msg.timestamp = _timestamp.load();
		msg.x = drone_start_position_in_plane(0); // in meters NED
		msg.y = -drone_start_position_in_plane(1); // in meters NED
		msg.z = -drone_start_position_in_plane(2); // in meters NED
		msg.yaw = -quatToEul(_alignment_pose.quaternion)(2); // rotation around z NED in radians
		msg.vx = 0.0; // m/s NED
		msg.vy = 0.0; // m/s NED
		msg.vz = 0.0; // m/s NED

		OffboardControl::publish_setpoint(msg);

		RCLCPP_INFO(this->get_logger(), "No selected powerline id, hovering in plane");

		return;
	}
	else
	{
		if (selected_pl_id_old != _selected_pl_id)
		{
			planned_path = false;
			
			path_pos_cnt = 0;

			planned_path_pcl->clear();

			selected_pl_id_old = _selected_pl_id;
		}

		
	}
	


	/* ----- (get tower position, calculate damper plane,) plan path ----- */
	
	

	// static pcl::PointCloud<pcl::PointXYZ>::Ptr planned_path_pcl (new pcl::PointCloud<pcl::PointXYZ>);


	if (!planned_path)
	{	

		// get opposite transform of plane
		// https://math.stackexchange.com/questions/3575361/difficulty-understanding-the-inverse-of-a-homogeneous-transformation-matrix

		plane_to_world = getInverseTransformMatrix(plane_position, plane_quaternion);

		homog_point_t drone_current_position; ///////////////@@@@@@@@@@@@@@@@@@@@@22
		drone_current_position(0) = _drone_pose.position(0);
		drone_current_position(1) = _drone_pose.position(1);
		drone_current_position(2) = _drone_pose.position(2);
		drone_current_position(3) = 1;


		homog_point_t drone_current_position_in_plane = plane_to_world * drone_current_position;

		// transform points in pointcloud

		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_plane_points (new pcl::PointCloud<pcl::PointXYZ>);

		pcl::transformPointCloud (*plane_line_points, *transformed_plane_points, plane_to_world);

		// auto trans_plane_msg = sensor_msgs::msg::PointCloud2();
		// OffboardControl::create_pointcloud_msg(transformed_plane_points, &trans_plane_msg);
		// _transformed_plane_pcl_pub->publish(trans_plane_msg);


		float smallest_y = 99999.0;
		float smallest_z = 99999.0;
		for (int i = 0; i < (int)transformed_plane_points->size(); i++)
		{
			if( transformed_plane_points->points[i].y < smallest_y) {
				smallest_y = transformed_plane_points->points[i].y;
			}

			if( transformed_plane_points->points[i].z < smallest_z) {
				smallest_z = transformed_plane_points->points[i].z;
			}
		}

		homog_transform_t zero_cloud_transform;

		orientation_t zero_rotation;
		zero_rotation(0) = 0.0;
		zero_rotation(1) = 0.0;
		zero_rotation(2) = 0.0;

		point_t diff_to_zero;
		diff_to_zero(0) = 0.0;
		diff_to_zero(1) = -smallest_y;
		diff_to_zero(2) = -smallest_z + lowest_powerline_z;

		zero_cloud_transform = getTransformMatrix((vector_t)diff_to_zero, eulToQuat(zero_rotation));

		drone_current_position_in_plane = zero_cloud_transform * drone_current_position_in_plane;

		pcl::transformPointCloud (*transformed_plane_points, *transformed_plane_points, zero_cloud_transform);
		
		// auto trans_plane_msg = sensor_msgs::msg::PointCloud2();
		// OffboardControl::create_pointcloud_msg(transformed_plane_points, &trans_plane_msg);
		// _transformed_plane_pcl_pub->publish(trans_plane_msg);

		// calculate start and goal points based on plane position and target powerline
		point2d_t point_start;
		point_start(0) = drone_current_position_in_plane(1);
		point_start(1) = drone_current_position_in_plane(2);


		// make goal a random line for testing
		// srand (time(NULL));
		int random_int = _selected_pl_id; //rand() % transformed_plane_points->size();

		point2d_t point_goal;
		point_goal(0) = transformed_plane_points->points[random_int].y;
		point_goal(1) = transformed_plane_points->points[random_int].z - 1.5; // 1.5m below random cable

		// make sure map contains both start and goal point
		// create pcl from found path

		pcl::PointCloud<pcl::PointXYZ>::Ptr path_pcl (new pcl::PointCloud<pcl::PointXYZ>);

		auto start = high_resolution_clock::now();

		OffboardControl::pathplanner(point_goal, point_start, transformed_plane_points, path_pcl); // inverse path?, therefore goal and start switched, stupid fix
		
		auto stop = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(stop - start);
		RCLCPP_INFO(this->get_logger(), "Pathplanning took: %f ms", (((float)duration.count()) / 1000));
		// while(1);

		transform_t inv_zero_transform = invertTransformMatrix(zero_cloud_transform); //getInverseTransformMatrix(zero_cloud_transform.block<3,1>(0,3), matToQuat(zero_cloud_transform.block<3,3>(0,0)));

		pcl::transformPointCloud (*path_pcl, *path_pcl, inv_zero_transform);

		transform_t inv_plane_to_world = invertTransformMatrix(plane_to_world); //getInverseTransformMatrix(plane_to_world.block<3,1>(0,3), matToQuat(plane_to_world.block<3,3>(0,0)));

		pcl::transformPointCloud (*path_pcl, *path_pcl, inv_plane_to_world);

		// visualize generated plane path
		auto plane_path_msg = nav_msgs::msg::Path();
		plane_path_msg.header.stamp = this->now();
		plane_path_msg.header.frame_id = "world";

		for (int i = 0; i < (int)path_pcl->size(); i++)
		{
		
			auto pose_msg = geometry_msgs::msg::PoseStamped();
			pose_msg.header.stamp = this->now();
			pose_msg.header.frame_id = "world";
			
			pose_msg.pose.position.x = path_pcl->points[i].x;
			pose_msg.pose.position.y = path_pcl->points[i].y;
			pose_msg.pose.position.z = path_pcl->points[i].z;

			plane_path_msg.poses.push_back(pose_msg);

		}

		// pcl::PointXYZ tmp_point;
		// tmp_point.x = plane_position(0);
		// tmp_point.y = plane_position(1);
		// tmp_point.z = plane_position(2) + _following_distance;
		// planned_path_pcl->push_back(tmp_point);
		*planned_path_pcl += *path_pcl;

		if (plane_path_msg.poses.size() > 1)
		{
			_plane_path_pub->publish(plane_path_msg);
		}
		else {
			RCLCPP_INFO(this->get_logger(), "\n \nNo valid path found\n");
		}

	}
 
	planned_path = true;


	// TODO:
	// - Fix that path slightly intersects into safety zone


	/* ----- fly pre-planned damper path ----- */

	// static int path_pos_cnt = 0;

	if (path_pos_cnt < (int)planned_path_pcl->size()) {

		px4_msgs::msg::TrajectorySetpoint msg{};
		msg.timestamp = _timestamp.load();
		msg.x = planned_path_pcl->points[path_pos_cnt].x; // in meters NED
		msg.y = -planned_path_pcl->points[path_pos_cnt].y; // in meters NED
		msg.z = -planned_path_pcl->points[path_pos_cnt].z; // in meters NED
		msg.yaw = -quatToEul(_alignment_pose.quaternion)(2); // rotation around z NED in radians

		// if (path_pos_cnt != 0 && path_pos_cnt < ((int)planned_path_pcl->size()-1)) // do unless first or last coordinate
		// {
		// 	float diff_x = planned_path_pcl->points[(path_pos_cnt+1)].x - planned_path_pcl->points[path_pos_cnt].x;
		// 	float diff_y = -(planned_path_pcl->points[(path_pos_cnt+1)].y - planned_path_pcl->points[path_pos_cnt].y);
		// 	float diff_z = -(planned_path_pcl->points[(path_pos_cnt+1)].z - planned_path_pcl->points[path_pos_cnt].z);

		// 	float traj_speed = 0.5;

		// 	msg.vx = (diff_x / abs(diff_x)) * traj_speed; // m/s NED
		// 	msg.vy = (diff_y / abs(diff_y)) * traj_speed; // m/s NED
		// 	msg.vz = (diff_z / abs(diff_z)) * traj_speed;// m/s NED

		// 	RCLCPP_INFO(this->get_logger(), "\n vx %f\n", msg.vx);
		// 	RCLCPP_INFO(this->get_logger(), "vy %f\n", msg.vy);
		// 	RCLCPP_INFO(this->get_logger(), "vz %f\n", msg.vz);

		// }
		

		OffboardControl::publish_setpoint(msg);

		// XYZ delta to goal waypoint (incl. ENU to NED)
		float x_diff = _drone_pose.position(0) - planned_path_pcl->points[path_pos_cnt].x;
		float y_diff = _drone_pose.position(1) - planned_path_pcl->points[path_pos_cnt].y;
		float z_diff = _drone_pose.position(2) - planned_path_pcl->points[path_pos_cnt].z;

		float threshold = 0.25;
		if ( x_diff < threshold &&
			 y_diff < threshold &&
			 z_diff < threshold &&
			 x_diff > -threshold &&
			 y_diff > -threshold &&
			 z_diff > -threshold
		)
		{
			path_pos_cnt++;
		}
		else {
			RCLCPP_INFO(this->get_logger(), "\n Delta to position %d/%d: \n X: %f \n Y: %f \n Z: %f \n", path_pos_cnt+1, (int)planned_path_pcl->size(), x_diff, y_diff, z_diff);
		}

		return;

	}


	// px4_msgs::msg::TrajectorySetpoint msg{};
	// msg.timestamp = _timestamp.load();
	// msg.x = NAN; // in meters NED
	// msg.y = NAN; // in meters NED
	// msg.z = NAN; // in meters NED
	// msg.yaw = -quatToEul(_alignment_pose.quaternion)(2); // rotation around z NED in radians
	// msg.vx = 0.0; // m/s NED
	// msg.vy = 0.0; // m/s NED
	// msg.vz = -0.25; // m/s NED
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();
	msg.x = planned_path_pcl->points.back().x; // in meters NED
	msg.y = -planned_path_pcl->points.back().y; // in meters NED
	msg.z = -planned_path_pcl->points.back().z; // in meters NED
	msg.yaw = -quatToEul(_alignment_pose.quaternion)(2); // rotation around z NED in radians
	msg.vx = 0.0; // m/s NED
	msg.vy = 0.0; // m/s NED
	msg.vz = 0.0; // m/s NED

	OffboardControl::publish_setpoint(msg);

	return; // remove if alignment needed


	/* ----- Alignment ----- */

	this->get_parameter("take_off_to_height", _takeoff_height);
	if(_takeoff_height > 1){
		static bool takeoff_print = false;
		if(takeoff_print == false){
			takeoff_print = true;
			RCLCPP_INFO(this->get_logger(), "\n \nTaking off to %f meters\n", _takeoff_height);
		}
		publish_takeoff_setpoint();
		return;
	}

	else if(_counter < 1000000000){
		if(_counter == 0 && _launch_with_debug > 0){
			RCLCPP_INFO(this->get_logger(), "\n \nBeginning alignment \n");
		}
		publish_tracking_setpoint();

		// RCLCPP_INFO(this->get_logger(), "Alignment pose:\n X %f \n Y: %f \n Z: %f",
		// 	_alignment_pose.position(0), _alignment_pose.position(1), _alignment_pose.position(2));	
			
		// RCLCPP_INFO(this->get_logger(), "Drone pose:\n X %f \n Y: %f \n Z: %f",		
		// 	_drone_pose.position(0), _drone_pose.position(1), _drone_pose.position(2));	

	}

	_counter++;

}


void OffboardControl::update_drone_pose() {

	geometry_msgs::msg::TransformStamped t;

	try {
		if (tf_buffer_->canTransform("world", "drone", tf2::TimePointZero))	{
			t = tf_buffer_->lookupTransform("world","drone",tf2::TimePointZero);
		}
		else {
			RCLCPP_INFO(this->get_logger(), "Can not transform");
			return;
		}
	} catch (const tf2::TransformException & ex) {
		RCLCPP_INFO(this->get_logger(), "Could not transform: %s", ex.what());
		return;
	}

	_drone_pose_mutex.lock(); {

		_drone_pose.position(0) = t.transform.translation.x;
		_drone_pose.position(1) = t.transform.translation.y;
		_drone_pose.position(2) = t.transform.translation.z;
		
		_drone_pose.quaternion(0) = t.transform.rotation.x;
		_drone_pose.quaternion(1) = t.transform.rotation.y;
		_drone_pose.quaternion(2) = t.transform.rotation.z;
		_drone_pose.quaternion(3) = t.transform.rotation.w;

	} _drone_pose_mutex.unlock();

}


void OffboardControl::update_alignment_pose(radar_cable_follower_msgs::msg::TrackedPowerlines::SharedPtr msg) {		
		
	if (msg->poses.size() < 1)
	{
		return;
	}


	float current_highest = 0;
	size_t highest_index = 0;
	int id = -1;

	_id_mutex.lock(); {
		id = _selected_ID;
	} _id_mutex.unlock();

	this->get_parameter("powerline_following_ID", id);

	for (size_t i = 0; i < msg->poses.size(); i++)
	{
		// find powerline corresponding to selected ID
		if (msg->ids[i] == id)
		{
			current_highest = msg->poses[i].position.z;
			highest_index = i;
			break;
		}
		
		// else find highest powerline
		if ( msg->poses[i].position.z > current_highest )
		{
			current_highest = msg->poses[i].position.z;
			highest_index = i;
		}
	}

	float tmp_follow_dist;
	this->get_parameter("powerline_following_distance", tmp_follow_dist);	

	_powerline_mutex.lock(); {	

		_alignment_pose.position(0) = msg->poses[highest_index].position.x;
		_alignment_pose.position(1) = msg->poses[highest_index].position.y;
		_alignment_pose.position(2) = msg->poses[highest_index].position.z + (float)tmp_follow_dist;

		_alignment_pose.quaternion(0) = msg->poses[highest_index].orientation.x;
		_alignment_pose.quaternion(1) = msg->poses[highest_index].orientation.y;
		_alignment_pose.quaternion(2) = msg->poses[highest_index].orientation.z;
		_alignment_pose.quaternion(3) = msg->poses[highest_index].orientation.w;

		// RCLCPP_INFO(this->get_logger(), "Alignment pose:\n X %f \n Y: %f \n Z: %f",
		// 	_alignment_pose.position(0), _alignment_pose.position(1), _alignment_pose.position(2));		

	} _powerline_mutex.unlock();

}


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send\n");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send\n");
}


/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = _timestamp.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;	
	_offboard_control_mode_publisher->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Should go to align with cable of choice there
 */
void OffboardControl::publish_tracking_setpoint() {

	static bool print_nothing_align_with_once = false;

	if (_alignment_pose.position(0) == 0 && 
		_alignment_pose.position(1) == 0 && 
		_alignment_pose.position(2) == 0)
	{

		static px4_msgs::msg::TrajectorySetpoint msg{};

		if (print_nothing_align_with_once == false)
		{		
			print_nothing_align_with_once = true;

			RCLCPP_INFO(this->get_logger(), "Nothing to align with - holding position");

			static pose_eul_t NWU_to_NED_pose;	

			NWU_to_NED_pose.position = _drone_pose.position; 
			NWU_to_NED_pose.orientation = quatToEul(_drone_pose.quaternion);
			NWU_to_NED_pose = pose_NWU_to_NED(NWU_to_NED_pose);

			msg.timestamp = _timestamp.load();
			msg.x = NWU_to_NED_pose.position(0); 		// in meters NED
			msg.y = NWU_to_NED_pose.position(1);
			msg.z = NWU_to_NED_pose.position(2);
			// YAW is cropped to 0-PI for some reason, uncrop to 0-2PI based on if ROLL is 0 or PI
			msg.yaw = (float)NWU_to_NED_pose.orientation(2);

		}

		OffboardControl::publish_setpoint(msg);
		return;
	}

	print_nothing_align_with_once = false;
	

	float pos_frac;
	this->get_parameter("pos_frac", pos_frac);

	float yaw_frac;
	this->get_parameter("yaw_frac", yaw_frac);
	static float tmp_follow_speed;
	this->get_parameter("powerline_following_speed", tmp_follow_speed);

	orientation_t target_yaw_eul;

	pose_eul_t publish_pose;

	_powerline_mutex.lock(); {
	_drone_pose_mutex.lock(); {
		
		// calculate fractional yaw positions (basic porportional control)
		target_yaw_eul = quatToEul(_alignment_pose.quaternion);

		float cur_yaw = quatToEul(_drone_pose.quaternion)(2); 
		float target_yaw = target_yaw_eul(2);
		
		if ( abs(cur_yaw - target_yaw) <= (float)M_PI )
		{
			target_yaw = cur_yaw + (target_yaw - cur_yaw)*yaw_frac;			
		}
		else 
		{
			float diff = (2*M_PI - target_yaw) + cur_yaw;

			target_yaw = cur_yaw - diff*yaw_frac;			
		}


		publish_pose.position(0) = _drone_pose.position(0) + (_alignment_pose.position(0) - _drone_pose.position(0))*pos_frac;
		publish_pose.position(1) = _drone_pose.position(1) + (_alignment_pose.position(1) - _drone_pose.position(1))*pos_frac;
		publish_pose.position(2) = _drone_pose.position(2) + (_alignment_pose.position(2) - _drone_pose.position(2))*pos_frac;		

		publish_pose.orientation(0) = 0.0;
		publish_pose.orientation(1) = 0.0;
		publish_pose.orientation(2) = target_yaw;


	} _drone_pose_mutex.unlock();
	} _powerline_mutex.unlock();

	point_t unit_x(
		1.0 * tmp_follow_speed,
		0.0,
		0.0
	);


	publish_pose = pose_NWU_to_NED(publish_pose);
	
	// rotate unit x (1,0,0) velocity to align with powerline direction
	rotation_matrix_t rot_mat = quatToMat(_alignment_pose.quaternion);
	point_t unit_velocity = rotateVector(rot_mat, unit_x);

	// rotate powerline direction velocity from NWU to NED frame
	static rotation_matrix_t R_NWU_to_NED = eulToR(orientation_t(-M_PI, 0, 0));
	unit_velocity = rotateVector(R_NWU_to_NED, unit_velocity);

	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();
	msg.x = publish_pose.position(0); // in meters NED
	msg.y = publish_pose.position(1); // in meters NED
	msg.z = publish_pose.position(2); // in meters NED
	msg.yaw = publish_pose.orientation(2); // rotation around z NED in radians
	msg.vx = unit_velocity(0); // m/s NED
	msg.vy = unit_velocity(1); // m/s NED
	msg.vz = unit_velocity(2); // m/s NED

	// RCLCPP_INFO(this->get_logger(), "Xv:%f Yv:%f Zv:%f", msg.velocity[0], msg.velocity[1], msg.velocity[2]);

	OffboardControl::publish_setpoint(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Drone should take off to _takeoff_height
 */
void OffboardControl::publish_takeoff_setpoint() {

	static pose_eul_t NWU_to_NED_pose;	

	if (_new_takeoff == true)
	{	
		// freeze takeoff setpoint
		_new_takeoff = false;
		NWU_to_NED_pose.position = _drone_pose.position; 
		NWU_to_NED_pose.orientation = quatToEul(_drone_pose.quaternion);
		NWU_to_NED_pose = pose_NWU_to_NED(NWU_to_NED_pose);
	}

	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();
	msg.x = NWU_to_NED_pose.position(0); 		// in meters NED
	msg.y = NWU_to_NED_pose.position(1);
	msg.z = - _takeoff_height;
	// YAW is cropped to 0-PI for some reason, uncrop to 0-2PI based on if ROLL is 0 or PI
	msg.yaw = (float)NWU_to_NED_pose.orientation(2);

	OffboardControl::publish_setpoint(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Drone should holds it pose
 */
void OffboardControl::publish_hold_setpoint() const {

	pose_eul_t NWU_to_NED_pose;
	NWU_to_NED_pose.position = _drone_pose.position; 
	NWU_to_NED_pose.orientation = quatToEul(_drone_pose.quaternion);	

	NWU_to_NED_pose = pose_NWU_to_NED(NWU_to_NED_pose);

	px4_msgs::msg::TrajectorySetpoint msg{}; // in meters NED
	msg.timestamp = _timestamp.load();
	msg.x = NWU_to_NED_pose.position(0); 		
	msg.y = NWU_to_NED_pose.position(1);
	msg.z = NWU_to_NED_pose.position(2);
	//YAW is cropped to 0-PI for some reason, uncrop to 0-2PI based on if ROLL is 0 or PI
	msg.yaw = (float)NWU_to_NED_pose.orientation(2);// + (float)NWU_to_NED_pose.orientation(0);

	// RCLCPP_INFO(this->get_logger(), "DRONE EUL:\n R:%f P:%f Y:%f ", NWU_to_NED_pose.orientation(0), NWU_to_NED_pose.orientation(1), NWU_to_NED_pose.orientation(2));

	OffboardControl::publish_setpoint(msg);
}


/**
 * @brief Publish a trajectory setpoint
 * and pose message
 */
void OffboardControl::publish_setpoint(px4_msgs::msg::TrajectorySetpoint msg) const {

	publish_offboard_control_mode();

	orientation_t eul (
		0.0,
		0.0,
		-msg.yaw // NED to NWU
	);

	quat_t quat = eulToQuat(eul);

	if (_launch_with_debug > 0)
	{	
		auto pose_msg = geometry_msgs::msg::PoseStamped();
		pose_msg.header.stamp = this->now();
		pose_msg.header.frame_id = "world";
		pose_msg.pose.orientation.x = quat(0);
		pose_msg.pose.orientation.y = quat(1);
		pose_msg.pose.orientation.z = quat(2);
		pose_msg.pose.orientation.w = quat(3);
		pose_msg.pose.position.x = msg.x;
		pose_msg.pose.position.y = -msg.y; // NED to NWU
		pose_msg.pose.position.z = -msg.z; // NED to NWU

		_follow_pose_pub->publish(pose_msg);
	}

	_trajectory_setpoint_publisher->publish(msg);
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = _timestamp.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	_vehicle_command_publisher->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
