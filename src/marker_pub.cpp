// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <sensor_msgs/msg/image.hpp>
#include "geometry.h"

// Debug
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

 // MISC includes
#include <cstdlib>
#include <stdlib.h> 
#include <chrono>
#include <math.h> 
#include <cmath> 
#include <vector>
#include <deque>
#include <string>
#include <numeric>
#include <mutex>



#define DEG_PER_RAD 57.2957795
#define RAD_PER_DEG 0.01745329
#define PI 3.14159265

using namespace std::chrono_literals;

//creates a MarkerPublisher class that subclasses the generic rclcpp::Node base class.
class MarkerPublisher : public rclcpp::Node
{
	public:
		MarkerPublisher() : Node("radar_pcl_filter_node") {

			_damper_plane_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pl_marker", 10);
			
			geometry_msgs::msg::TransformStamped drone_tf;

			_timer_pl = this->create_wall_timer(500ms, std::bind(&MarkerPublisher::offset_tower_plane_and_points, this));
		}

		~MarkerPublisher() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down radar_pointcloud_filter node..");
		}

	private:

		rclcpp::TimerBase::SharedPtr _timer_pl;

		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _damper_plane_pub;

		void offset_tower_plane_and_points();
};




void MarkerPublisher::offset_tower_plane_and_points() {

	/*
	- Define plane offset X m from tower in PL direction
	- (Make sure there are cables on this side of tower)
	- Find points closest to plane, project onto plane
		- Iterate over all points in cloud
		- Check XY distance from plane projected onto ground
		- Separate into heights (maybe 0.5m?)
	- Create new pcl message from projected points, publish to OffboardController
	- Add interactivity to marker to be able to press it to fly to it
	*/


	// visualize with plane marker
	visualization_msgs::msg::Marker marker;
	marker.header = std_msgs::msg::Header();
	marker.header.stamp = this->now();
	marker.header.frame_id = "world";

	marker.ns = "damper_plane";
	marker.id = 0;

	marker.type = visualization_msgs::msg::Marker::CYLINDER;

	marker.action = visualization_msgs::msg::Marker::ADD;

	orientation_t tmp_rpy; // not sure why this order, probably marker definition
	tmp_rpy(2) = 0;
	tmp_rpy(0) = 0;
	tmp_rpy(1) = 0;

  pose_t tower_pose;
	tower_pose.quaternion = eulToQuat(tmp_rpy);

	marker.pose.orientation.x = tower_pose.quaternion(0);
	marker.pose.orientation.y = tower_pose.quaternion(1);
	marker.pose.orientation.z = tower_pose.quaternion(2);
	marker.pose.orientation.w = tower_pose.quaternion(3);
	marker.pose.position.x = 3;
	marker.pose.position.y = 3;
	marker.pose.position.z = 3; // to fit plane from ground to above line

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 4;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.6;

	marker.lifetime = rclcpp::Duration::from_seconds(0);







  	// visualize with plane marker
	visualization_msgs::msg::Marker marker2;
	marker2.header = std_msgs::msg::Header();
	marker2.header.stamp = this->now();
	marker2.header.frame_id = "world";

	marker2.ns = "damper_plane";
	marker2.id = 1;

	marker2.type = visualization_msgs::msg::Marker::CYLINDER;

	marker2.action = visualization_msgs::msg::Marker::ADD;

	orientation_t tmp_rpy2; // not sure why this order, probably marker2 definition
	tmp_rpy2(2) = 0;
	tmp_rpy2(0) = 0;
	tmp_rpy2(1) = 0;

	tower_pose.quaternion = eulToQuat(tmp_rpy);

	marker2.pose.orientation.x = tower_pose.quaternion(0);
	marker2.pose.orientation.y = tower_pose.quaternion(1);
	marker2.pose.orientation.z = tower_pose.quaternion(2);
	marker2.pose.orientation.w = tower_pose.quaternion(3);
	marker2.pose.position.x = -3;
	marker2.pose.position.y = 3;
	marker2.pose.position.z = 3; // to fit plane from ground to above line

	// Set the scale of the marker2 -- 1x1x1 here means 1m on a side
	marker2.scale.x = 1;
	marker2.scale.y = 1;
	marker2.scale.z = 4;
	// Set the color -- be sure to set alpha to something non-zero!
	marker2.color.r = 0.0f;
	marker2.color.g = 0.0f;
	marker2.color.b = 1.0f;
	marker2.color.a = 0.6;

	marker2.lifetime = rclcpp::Duration::from_seconds(0);


	visualization_msgs::msg::MarkerArray marker_array;
	marker_array.markers.push_back(marker);
	marker_array.markers.push_back(marker2);

	_damper_plane_pub->publish(marker_array);


}


	
			
int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MarkerPublisher>());

	rclcpp::shutdown();
	return 0;
}
