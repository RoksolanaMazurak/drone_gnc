#include <gnc_functions.hpp>

//include API 

int main(int argc, char** argv)
{
 //initialize ros 
 rclcpp::init(argc, argv); // Initialize the ROS 2 node

  // Create a node
    auto node = rclcpp::Node::make_shared("gnc_node");
 
 //initialize control publisher/subscribers
 init_publisher_subscriber(node);

   // wait for FCU connection
 wait4connect();

 //wait for used to switch to mode GUIDED
 wait4start();

 //create local reference frame 
 initialize_local_frame();

 //request takeoff
 takeoff(3);

 //specify some waypoints 
 std::vector<gnc_api_waypoint> waypointList;
 gnc_api_waypoint nextWayPoint;
 nextWayPoint.x = 0;
 nextWayPoint.y = 0;
 nextWayPoint.z = 3;
 nextWayPoint.psi = 0;
 waypointList.push_back(nextWayPoint);
 nextWayPoint.x = 5;
 nextWayPoint.y = 0;
 nextWayPoint.z = 3;
 nextWayPoint.psi = -90;
 waypointList.push_back(nextWayPoint);
 nextWayPoint.x = 5;
 nextWayPoint.y = 5;
 nextWayPoint.z = 3;
 nextWayPoint.psi = 0;
 waypointList.push_back(nextWayPoint);
 nextWayPoint.x = 0;
 nextWayPoint.y = 5;
 nextWayPoint.z = 3;
 nextWayPoint.psi = 90;
 waypointList.push_back(nextWayPoint);
 nextWayPoint.x = 0;
 nextWayPoint.y = 0;
 nextWayPoint.z = 3;
 nextWayPoint.psi = 180;
 waypointList.push_back(nextWayPoint);
 nextWayPoint.x = 0;
 nextWayPoint.y = 0;
 nextWayPoint.z = 3;
 nextWayPoint.psi = 0;
 waypointList.push_back(nextWayPoint);


 //specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
 rclcpp::Rate rate(2.0); // Set the desired loop rate
 int counter = 0;
 while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    rate.sleep();

    if (check_waypoint_reached(.3) == 1)
    {
      if (counter < waypointList.size())
      {
        set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
        counter++;
      }
      else
      {
        // Land after all waypoints are reached
        land();
      }
    }
  }
 return 0;
}
