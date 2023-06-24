#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/waypoint_pull.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/waypoint_set_current.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <unistd.h>
#include <vector>
#include <rclcpp/duration.hpp>
#include <iostream>
#include <string>

mavros_msgs::msg::State current_state_g;
nav_msgs::msg::Odometry current_pose_g;
geometry_msgs::msg::Pose correction_vector_g;
geometry_msgs::msg::Point local_offset_pose_g;
geometry_msgs::msg::PoseStamped waypoint_g;

float current_heading_g;
float local_offset_g;
float correction_heading_g = 0;
float local_desired_heading_g; 

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub;
rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr global_lla_pos_pub;
rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr global_lla_pos_pub_raw;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr currentPos;
rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client;
rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client;
rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client;
rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedPtr auto_waypoint_pull_client;
rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr auto_waypoint_push_client;
rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedPtr auto_waypoint_set_current_client;

struct gnc_api_waypoint{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	float psi; ///< rotation about the third axis of your reference frame
};

void state_cb(const mavros_msgs::msg::State::ConstPtr& msg)
{
  current_state_g = *msg;
}

geometry_msgs::msg::Point enu_2_local(nav_msgs::msg::Odometry current_pose_enu)
{
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI/180);
  geometry_msgs::msg::Point current_pos_local;
  current_pos_local.x = x*cos((local_offset_g - 90)*deg2rad) - y*sin((local_offset_g - 90)*deg2rad);
  current_pos_local.y = x*sin((local_offset_g - 90)*deg2rad) + y*cos((local_offset_g - 90)*deg2rad);
  current_pos_local.z = z;

  return current_pos_local;

  //ROS_INFO("Local position %f %f %f",X, Y, Z);
}

//get current position of drone
void pose_cb(const nav_msgs::msg::Odometry::ConstPtr& msg)
{
  current_pose_g = *msg;
  enu_2_local(current_pose_g);
  float q0 = current_pose_g.pose.pose.orientation.w;
  float q1 = current_pose_g.pose.pose.orientation.x;
  float q2 = current_pose_g.pose.pose.orientation.y;
  float q3 = current_pose_g.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  //ROS_INFO("Current Heading %f ENU", psi*(180/M_PI));
  //Heading is in ENU
  //IS YAWING COUNTERCLOCKWISE POSITIVE?
  current_heading_g = psi*(180/M_PI) - local_offset_g;
  //ROS_INFO("Current Heading %f origin", current_heading_g);
  //ROS_INFO("x: %f y: %f z: %f", current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, current_pose_g.pose.pose.position.z);
}

geometry_msgs::msg::Point get_current_location()
{
	geometry_msgs::msg::Point current_pos_local;
	current_pos_local = enu_2_local(current_pose_g);
	return current_pos_local;

}
float get_current_heading()
{
	return current_heading_g;
}

//set orientation of the drone (drone should always be level) 
// Heading input should match the ENU coordinate system
/**
\ingroup control_functions
This function is used to specify the drone’s heading in the local reference frame. Psi is a counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
void set_heading(float heading)
{
  local_desired_heading_g = heading; 
  heading = heading + correction_heading_g + local_offset_g;
  
  RCLCPP_INFO_STREAM(rclcpp::get_logger("logger_name"), "Desired Heading " << local_desired_heading_g);
  float yaw = heading*(M_PI/180);
  float pitch = 0;
  float roll = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  waypoint_g.pose.orientation.w = qw;
  waypoint_g.pose.orientation.x = qx;
  waypoint_g.pose.orientation.y = qy;
  waypoint_g.pose.orientation.z = qz;
}

// set position to fly to in the local frame
/**
\ingroup control_functions
This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
void set_destination(float x, float y, float z, float psi)
{
	set_heading(psi);
	//transform map to local
	float deg2rad = (M_PI/180);
	float Xlocal = x*cos((correction_heading_g + local_offset_g - 90)*deg2rad) - y*sin((correction_heading_g + local_offset_g - 90)*deg2rad);
	float Ylocal = x*sin((correction_heading_g + local_offset_g - 90)*deg2rad) + y*cos((correction_heading_g + local_offset_g - 90)*deg2rad);
	float Zlocal = z;

	x = Xlocal + correction_vector_g.position.x + local_offset_pose_g.x;
	y = Ylocal + correction_vector_g.position.y + local_offset_pose_g.y;
	z = Zlocal + correction_vector_g.position.z + local_offset_pose_g.z;
	RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Destination set to x: %f y: %f z: %f origin frame", x, y, z);


	waypoint_g.pose.position.x = x;
	waypoint_g.pose.position.y = y;
	waypoint_g.pose.position.z = z;

	local_pos_pub->publish(waypoint_g);	
}
void set_destination_lla(float lat, float lon, float alt, float heading)
{
	geographic_msgs::msg::GeoPoseStamped lla_msg;
	// mavros_msgs::GlobalPositionTarget 
	lla_msg.pose.position.latitude = lat;
	lla_msg.pose.position.longitude = lon;
	lla_msg.pose.position.altitude = alt;
	float yaw = heading*(M_PI/180);
	float pitch = 0;
	float roll = 0;

	float cy = cos(yaw * 0.5);
	float sy = sin(yaw * 0.5);
	float cr = cos(roll * 0.5);
	float sr = sin(roll * 0.5);
	float cp = cos(pitch * 0.5);
	float sp = sin(pitch * 0.5);

	float qw = cy * cr * cp + sy * sr * sp;
	float qx = cy * sr * cp - sy * cr * sp;
	float qy = cy * cr * sp + sy * sr * cp;
	float qz = sy * cr * cp - cy * sr * sp;

	lla_msg.pose.orientation.w = qw;
	lla_msg.pose.orientation.x = qx;
	lla_msg.pose.orientation.y = qy;
	lla_msg.pose.orientation.z = qz;
	global_lla_pos_pub->publish(lla_msg);
}

void set_destination_lla_raw(float lat, float lon, float alt, float heading)
{
	mavros_msgs::msg::GlobalPositionTarget lla_msg;
	lla_msg.coordinate_frame = lla_msg.FRAME_GLOBAL_TERRAIN_ALT;
	lla_msg.type_mask = lla_msg.IGNORE_VX | lla_msg.IGNORE_VY | lla_msg.IGNORE_VZ | lla_msg.IGNORE_AFX | lla_msg.IGNORE_AFY | lla_msg.IGNORE_AFZ | lla_msg.IGNORE_YAW | lla_msg.IGNORE_YAW_RATE ; 
	lla_msg.latitude = lat;
	lla_msg.longitude = lon;
	lla_msg.altitude = alt;
	// lla_msg.yaw = heading;
	global_lla_pos_pub_raw->publish(lla_msg);
}
// **
// \ingroup control_functions
// Wait for connect is a function that will hold the program until communication with the FCU is established.
// @returns 0 - connected to fcu 
// @returns -1 - failed to connect to drone
// **
// /
int wait4connect(rclcpp::Node::SharedPtr node, mavros_msgs::msg::State::SharedPtr current_state)
{
  RCLCPP_INFO(node->get_logger(), "Waiting for FCU connection");

  // Wait for FCU connection
  while (rclcpp::ok() && !current_state->connected)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (current_state->connected)
  {
    RCLCPP_INFO(node->get_logger(), "Connected to FCU");
    return 0;
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Error connecting to drone");
    return -1;
  }
}

/**
\ingroup control_functions
Wait for strat will hold the program until the user signals the FCU to enther mode guided. This is typically done from a switch on the safety pilot’s remote or from the ground control station.
@returns 0 - mission started
@returns -1 - failed to start mission
*/
int wait4start(mavros_msgs::msg::State::SharedPtr current_state)
{
  RCLCPP_INFO(rclcpp::get_logger("wait4start"), "Waiting for user to set mode to GUIDED");
  rclcpp::Rate loop_rate(10); // Adjust the loop rate as per your requirements

  while (rclcpp::ok() && current_state->mode != "GUIDED")
  {
    rclcpp::spin_some();
    loop_rate.sleep();
  }

  if (current_state->mode == "GUIDED")
  {
    RCLCPP_INFO(rclcpp::get_logger("wait4start"), "Mode set to GUIDED. Mission starting");
    return 0;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("wait4start"), "Error starting mission!!");
    return -1;
  }
}
/**
\ingroup control_functions
This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to.
@returns 0 - frame initialized
*/

int initialize_local_frame()
{
    RCLCPP_INFO(rclcpp::get_logger("initialize_local_frame"), "Initializing local coordinate system");
    float local_offset_g = 0.0;
    geometry_msgs::msg::PoseStamped current_pose_g;
    geometry_msgs::msg::PoseStamped local_offset_pose_g;

    for (int i = 1; i <= 30; i++)
    {
        rclcpp::spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        float q0 = current_pose_g.pose.orientation.w;
        float q1 = current_pose_g.pose.orientation.x;
        float q2 = current_pose_g.pose.orientation.y;
        float q3 = current_pose_g.pose.orientation.z;
        float psi = atan2((2 * (q0 * q3 + q1 * q2)), (1 - 2 * (pow(q2, 2) + pow(q3, 2)))); // yaw

        local_offset_g += psi * (180 / M_PI);

        local_offset_pose_g.pose.position.x += current_pose_g.pose.position.x;
        local_offset_pose_g.pose.position.y += current_pose_g.pose.position.y;
        local_offset_pose_g.pose.position.z += current_pose_g.pose.position.z;
    }

    local_offset_g /= 30;
    local_offset_pose_g.pose.position.x /= 30;
    local_offset_pose_g.pose.position.y /= 30;
    local_offset_pose_g.pose.position.z /= 30;

    RCLCPP_INFO(rclcpp::get_logger("initialize_local_frame"), "Coordinate offset set");
    RCLCPP_INFO(rclcpp::get_logger("initialize_local_frame"), "The X' axis is facing: %f", local_offset_g);

    return 0;
}

// int arm()
// {
//   // Initialize first waypoint of mission
//   set_destination(0, 0, 0, 0);
//   for (int i = 0; i < 100; i++)
//   {
//     local_pos_pub->publish(waypoint_g);
//     rclcpp::spin_some()
//     rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1e7))); // Sleep for 0.01 seconds
//   }

//   // Arming
//   RCLCPP_INFO(rclcpp::get_logger("arm"), "Arming drone");
//   auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
//   arm_request->value = true;

//   auto result_future = arming_client->async_send_request(arm_request);
//   if (rclcpp::spin_until_future_complete(arming_client->get_node(), result_future) ==
//       rclcpp::FutureReturnCode::SUCCESS)
//   {
//     if (result_future.get()->success)
//     {
//       RCLCPP_INFO(rclcpp::get_logger("arm"), "Arming Successful");
//       return 0;
//     }
//     else
//     {
//       RCLCPP_INFO(rclcpp::get_logger("arm"), "Arming failed with %d", result_future.get()->success);
//       return -1;
//     }
//   }
//   else
//   {
//     RCLCPP_INFO(rclcpp::get_logger("arm"), "Failed to call arming service");
//     return -1;
//   }
// }
/**
\ingroup control_functions
The takeoff function will arm the drone and put the drone in a hover above the initial position. 
@returns 0 - nominal takeoff 
@returns -1 - failed to arm 
@returns -2 - failed to takeoff
*/
int takeoff(float takeoff_alt)
{
    // Initialize first waypoint of mission
    set_destination(0, 0, takeoff_alt, 0);
    for (int i = 0; i < 100; i++)
    {
        local_pos_pub->publish(waypoint_g);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Arming
    RCLCPP_INFO(rclcpp::get_logger("takeoff"), "Arming drone");
    mavros_msgs::srv::CommandBool arm_request;
    arm_request.arm= true;
    while (!current_state_g.armed && !arm_request.value && rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        arm_client->async_send_request(arm_request);
        local_pos_pub.publish(waypoint_g);
        rclcpp::spin_some(node);
    }

    if (arm_request.value)
    {
        RCLCPP_INFO(rclcpp::get_logger("takeoff"), "Arming Successful");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("takeoff"), "Arming failed");
        return -1;
    }

    // Request takeoff
    RCLCPP_INFO(rclcpp::get_logger("takeoff"), "Requesting takeoff");
    mavros_msgs::srv::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = takeoff_alt;
    if (takeoff_client->async_send_request(srv_takeoff))
    {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        RCLCPP_INFO(rclcpp::get_logger("takeoff"), "Takeoff request sent");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("takeoff"), "Failed to send takeoff request");
        return -2;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));
    return 0;
}


/**
\ingroup control_functions
This function returns an int of 1 or 0. THis function can be used to check when to request the next waypoint in the mission. 
@return 1 - waypoint reached 
@return 0 - waypoint not reached
*/
int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01)
{
	local_pos_pub->publish(waypoint_g);
	
	//check for correct position 
	float deltaX = abs(waypoint_g.pose.position.x - current_pose_g.pose.pose.position.x);
    float deltaY = abs(waypoint_g.pose.position.y - current_pose_g.pose.pose.position.y);
    float deltaZ = 0; //abs(waypoint_g.pose.position.z - current_pose_g.pose.pose.position.z);
    float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
    // ROS_INFO("dMag %f", dMag);
    // ROS_INFO("current pose x %F y %f z %f", (current_pose_g.pose.pose.position.x), (current_pose_g.pose.pose.position.y), (current_pose_g.pose.pose.position.z));
    // ROS_INFO("waypoint pose x %F y %f z %f", waypoint_g.pose.position.x, waypoint_g.pose.position.y,waypoint_g.pose.position.z);
    //check orientation
    float cosErr = cos(current_heading_g*(M_PI/180)) - cos(local_desired_heading_g*(M_PI/180));
    float sinErr = sin(current_heading_g*(M_PI/180)) - sin(local_desired_heading_g*(M_PI/180));
    
    float headingErr = sqrt( pow(cosErr, 2) + pow(sinErr, 2) );

    // ROS_INFO("current heading %f", current_heading_g);
    // ROS_INFO("local_desired_heading_g %f", local_desired_heading_g);
    // ROS_INFO("current heading error %f", headingErr);

    if( dMag < pos_tolerance && headingErr < heading_tolerance)
	{
		return 1;
	}else{
		return 0;
	}
}
/**
\ingroup control_functions
this function changes the mode of the drone to a user specified mode. This takes the mode as a string. ex. set_mode("GUIDED")
@returns 1 - mode change successful
@returns 0 - mode change not successful
*/
// int set_mode(std::string mode)
// {
// 	mavros_msgs::SetMode srv_setMode;
//     srv_setMode.request.base_mode = 0;
//     srv_setMode.request.custom_mode = mode.c_str();
//     if(set_mode_client.call(srv_setMode)){
//       ROS_INFO("setmode send ok");
// 	  return 0;
//     }else{
//       ROS_ERROR("Failed SetMode");
//       return -1;
//     }
// }
/**
\ingroup control_functions
this function changes the mode of the drone to land
@returns 1 - mode change successful
@returns 0 - mode change not successful
*/
int land()
{
  auto srv_land = std::make_shared<mavros_msgs::srv::CommandTOL>();
  if (land_client->call(srv_land) && srv_land->response.success)
  {
    RCLCPP_INFO(rclcpp::get_logger("land"), "Land sent %d", srv_land->response.success);
    return 0;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("land"), "Landing failed");
    return -1;
  }
}


/**
\ingroup control_functions
This function is used to change the speed of the vehicle in guided mode. it takes the speed in meters per second as a float as the input
@returns 0 for success
*/
// int set_speed(float speed__mps)
// {
// 	mavros_msgs::CommandLong speed_cmd;
// 	speed_cmd.request.command = 178;
// 	speed_cmd.request.param1 = 1; // ground speed type 
// 	speed_cmd.request.param2 = speed__mps;
// 	speed_cmd.request.param3 = -1; // no throttle change
// 	speed_cmd.request.param4 = 0; // absolute speed
// 	ROS_INFO("setting speed to %f", speed__mps);
// 	if(command_client.call(speed_cmd))
// 	{
// 		ROS_INFO("change speed command succeeded %d", speed_cmd.response.success);
// 		return 0;
// 	}else{
// 		ROS_ERROR("change speed command failed %d", speed_cmd.response.success);
// 		ROS_ERROR("change speed result was %d ", speed_cmd.response.result);
// 		return -1;
// 	}
// 	ROS_INFO("change speed result was %d ", speed_cmd.response.result);
// 	return 0;
// }
// int auto_set_current_waypoint(int seq)
// {
// 	mavros_msgs::WaypointSetCurrent wp_set_cur_msg;
// 	wp_set_cur_msg.request.wp_seq = seq;
// 	ROS_INFO("setting current wp to wp # %d", seq);
// 	if(auto_waypoint_set_current_client.call(wp_set_cur_msg))
// 	{
// 		ROS_INFO("set current wp secceeded %d", wp_set_cur_msg.response.success);
// 	}else{
// 		ROS_ERROR("set current wp failed %d", wp_set_cur_msg.response.success);
// 	}
// 	return 0;
// }
/**
\ingroup control_functions
used to set yaw when running lla waypoint missions
param1: Angle				target angle, 0 is north																			deg
param2: Angular Speed		angular speed																						deg/s
param3: Direction			direction: -1: counter clockwise, 1: clockwise					min: -1 max:1 increment:2	
param4: Relative			0: absolute angle, 1: relative offset							min:0 max:1 increment:1	
@returns 0 for success
*/
// int set_yaw(float angle, float speed, float dir, float absolute_rel)
// {
//     mavros_msgs::srv::CommandLong yaw_msg;
//     yaw_msg.request.command = 115;
//     yaw_msg.request.param1 = angle; // target angle 
//     yaw_msg.request.param2 = speed; //target speed
//     yaw_msg.request.param3 = dir; 
//     yaw_msg.request.param4 = absolute_rel; 
//     ROS_INFO("Setting the yaw angle to %f [deg]", angle);
//     if(command_client.call(yaw_msg))
//     {
//         ROS_INFO("yaw angle set returned %d", yaw_msg.response.success);
//         return 0;
//     }else{
//         ROS_ERROR("setting yaw angle failed %d", yaw_msg.response.success);
//         return -1;
//     }
// 	if(absolute_rel == 0 )
// 	{
// 		ROS_INFO("Yaw angle set at %d ", yaw_msg.response.result);
// 	}else{
// 		ROS_INFO("Yaw angle set at %d relative to current heading", yaw_msg.response.result);
// 	}
    
//     return 0;
// }
// int takeoff_global(float lat, float lon, float alt)
// {
//     mavros_msgs::CommandTOL srv_takeoff_global;
//     srv_takeoff_global.request.min_pitch = 0;
//     //srv_takeoff_global.request.yaw = heading; //check yaw angle
//     srv_takeoff_global.request.latitude = lat;
//     srv_takeoff_global.request.longitude = lon;
//     srv_takeoff_global.request.altitude = alt;
        
//     if(takeoff_client.call(srv_takeoff_global)){
//         sleep(3);
//         ROS_INFO("takeoff sent at the provided GPS coordinates %d", srv_takeoff_global.response.success);
//     }
//     else
//     {
//         ROS_ERROR("Failed Takeoff %d", srv_takeoff_global.response.success);
        
//     }
//     sleep(2);
//     return 0;
// }
/**
\ingroup control_functions
This function is called at the beginning of a program and will start of the communication links to the FCU. The function requires the program's ros nodehandle as an input 
@returns n/a
*/
int init_publisher_subscriber(rclcpp::Node controlnode)
{
	std::string ros_namespace;
	if (!controlnode.has_parameter("namespace"))
	{

		RCLCPP_INFO(rclcpp::get_logger("default"), "using default namespace");

	}else{
		controlnode.get_parameter("namespace", ros_namespace);
		RCLCPP_INFO(rclcpp::get_logger("square_node"), "using namespace %s", ros_namespace.c_str());

	}
	local_pos_pub = controlnode.create_publisher<geometry_msgs::msg::PoseStamped>((ros_namespace + "/mavros/setpoint_position/local").c_str(), 10);
	global_lla_pos_pub = controlnode.create_publisher<geographic_msgs::msg::GeoPoseStamped>((ros_namespace + "/mavros/setpoint_position/global").c_str(), 10);
	global_lla_pos_pub_raw = controlnode.create_publisher<mavros_msgs::msg::GlobalPositionTarget>((ros_namespace + "/mavros/setpoint_raw/global").c_str(), 10);
	currentPos = controlnode.create_subscription<nav_msgs::msg::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);
	state_sub = controlnode.create_subscription<mavros_msgs::msg::State>((ros_namespace + "/mavros/state").c_str(), 10, state_cb);
	arming_client = controlnode.create_client<mavros_msgs::srv::CommandBool>((ros_namespace + "/mavros/cmd/arming").c_str());
	land_client = controlnode.create_client<mavros_msgs::srv::CommandTOL>((ros_namespace + "/mavros/cmd/land").c_str());
	set_mode_client = controlnode.create_client<mavros_msgs::srv::SetMode>((ros_namespace + "/mavros/set_mode").c_str());
	takeoff_client = controlnode.create_client<mavros_msgs::srv::CommandTOL>((ros_namespace + "/mavros/cmd/takeoff").c_str());
	command_client = controlnode.create_client<mavros_msgs::srv::CommandLong>((ros_namespace + "/mavros/cmd/command").c_str());
	auto_waypoint_pull_client = controlnode.create_client<mavros_msgs::srv::WaypointPull>((ros_namespace + "/mavros/mission/pull").c_str());
	auto_waypoint_push_client = controlnode.create_client<mavros_msgs::srv::WaypointPush>((ros_namespace + "/mavros/mission/push").c_str());
	auto_waypoint_set_current_client = controlnode.create_client<mavros_msgs::srv::WaypointSetCurrent>((ros_namespace + "/mavros/mission/set_current").c_str());
	return 0;
}