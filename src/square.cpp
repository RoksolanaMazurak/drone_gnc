#include "rclcpp/rclcpp.hpp"
#include <gnc_functions.hpp>
#include <cmath>

#define PI 3.14159265

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("gnc_node");

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

 int r = 5;
 int d = 3;
 int z = 3;

 nextWayPoint.x = 0;
 nextWayPoint.y = 0;
 nextWayPoint.z = z;
 nextWayPoint.psi = 0;
 waypointList.push_back(nextWayPoint);
 
 nextWayPoint.x = 0;
 nextWayPoint.y = r+d;
 nextWayPoint.z = z;
 nextWayPoint.psi = 180;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = tan(18*PI/180)*(r+d);
 nextWayPoint.y = r+d;
 nextWayPoint.z = z;
 nextWayPoint.psi = 180;
 waypointList.push_back(nextWayPoint);
 
 nextWayPoint.x = tan(18*PI/180)*(r+d);
 nextWayPoint.y = r+d;
 nextWayPoint.z = z;
 nextWayPoint.psi = 144;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = tan(18*PI/180)*(r+d) + 2*(r+d)*tan(18*PI/180)*sin(54*PI/180);
 nextWayPoint.y = r+d - 4*(r+d)*pow(sin(18*PI/180), 2);
 nextWayPoint.z = z;
 nextWayPoint.psi = 144;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = tan(18*PI/180)*(r+d) + 2*(r+d)*tan(18*PI/180)*sin(54*PI/180);
 nextWayPoint.y = r+d - 4*(r+d)*pow(sin(18*PI/180), 2);
 nextWayPoint.z = z;
 nextWayPoint.psi = 108;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = tan(18*PI/180)*(r+d) + 2*(r+d)*tan(18*PI/180)*sin(54*PI/180) + 2*(r+d)*tan(18*PI/180)*sin(18*PI/180);
 nextWayPoint.y = r+d - 4*(r+d)*pow(sin(18*PI/180), 2) - 2*(r+d)*sin(18*PI/180);
 nextWayPoint.z = z;
 nextWayPoint.psi = 108;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = tan(18*PI/180)*(r+d) + 2*(r+d)*tan(18*PI/180)*sin(54*PI/180) + 2*(r+d)*tan(18*PI/180)*sin(18*PI/180);
 nextWayPoint.y = r+d - 4*(r+d)*pow(sin(18*PI/180), 2) - 2*(r+d)*sin(18*PI/180);
 nextWayPoint.z = z;
 nextWayPoint.psi = 72;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = tan(18*PI/180)*(r+d) + 2*(r+d)*tan(18*PI/180)*sin(54*PI/180);
 nextWayPoint.y = r+d - 4*(r+d)*pow(sin(18*PI/180), 2) - 2*2*(r+d)*sin(18*PI/180);
 nextWayPoint.z = z;
 nextWayPoint.psi = 72;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = tan(18*PI/180)*(r+d) + 2*(r+d)*tan(18*PI/180)*sin(54*PI/180);
 nextWayPoint.y = r+d - 4*(r+d)*pow(sin(18*PI/180), 2) - 2*2*(r+d)*sin(18*PI/180);
 nextWayPoint.z = z;
 nextWayPoint.psi = 36;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = tan(18*PI/180)*(r+d);
 nextWayPoint.y = r+d - 2*4*(r+d)*pow(sin(18*PI/180), 2) - 2*2*(r+d)*sin(18*PI/180);
 nextWayPoint.z = z;
 nextWayPoint.psi = 36;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = tan(18*PI/180)*(r+d);
 nextWayPoint.y = r+d - 2*4*(r+d)*pow(sin(18*PI/180), 2) - 2*2*(r+d)*sin(18*PI/180);
 nextWayPoint.z = z;
 nextWayPoint.psi = 0;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = -tan(18*PI/180)*(r+d);
 nextWayPoint.y = r+d - 2*4*(r+d)*pow(sin(18*PI/180), 2) - 2*2*(r+d)*sin(18*PI/180);
 nextWayPoint.z = z;
 nextWayPoint.psi = 0;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = -tan(18*PI/180)*(r+d);
 nextWayPoint.y = r+d - 2*4*(r+d)*pow(sin(18*PI/180), 2) - 2*2*(r+d)*sin(18*PI/180);
 nextWayPoint.z = z;
 nextWayPoint.psi = -36;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = -tan(18*PI/180)*(r+d) - 2*(r+d)*tan(18*PI/180)*sin(54*PI/180);
 nextWayPoint.y = r+d - 4*(r+d)*pow(sin(18*PI/180), 2) - 2*2*(r+d)*sin(18*PI/180);
 nextWayPoint.z = z;
 nextWayPoint.psi = -36;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = -tan(18*PI/180)*(r+d) - 2*(r+d)*tan(18*PI/180)*sin(54*PI/180);
 nextWayPoint.y = r+d - 4*(r+d)*pow(sin(18*PI/180), 2) - 2*2*(r+d)*sin(18*PI/180);
 nextWayPoint.z = z;
 nextWayPoint.psi = -72;
 waypointList.push_back(nextWayPoint);
nextWayPoint.x = -tan(18*PI/180)*(r+d) - 2*(r+d)*tan(18*PI/180)*sin(54*PI/180) - 2*(r+d)*tan(18*PI/180)*sin(18*PI/180);
 nextWayPoint.y = r+d - 4*(r+d)*pow(sin(18*PI/180), 2) - 2*(r+d)*sin(18*PI/180);
 nextWayPoint.z = z;
 nextWayPoint.psi = -108;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = -tan(18*PI/180)*(r+d) - 2*(r+d)*tan(18*PI/180)*sin(54*PI/180);
 nextWayPoint.y = r+d - 4*(r+d)*pow(sin(18*PI/180), 2);
 nextWayPoint.z = z;
 nextWayPoint.psi = -108;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = -tan(18*PI/180)*(r+d) - 2*(r+d)*tan(18*PI/180)*sin(54*PI/180);
 nextWayPoint.y = r+d - 4*(r+d)*pow(sin(18*PI/180), 2);
 nextWayPoint.z = z;
 nextWayPoint.psi = -144;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = -tan(18*PI/180)*(r+d);
 nextWayPoint.y = r+d;
 nextWayPoint.z = z;
 nextWayPoint.psi = -144;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = -tan(18*PI/180)*(r+d);
 nextWayPoint.y = r+d;
 nextWayPoint.z = z;
 nextWayPoint.psi = -180;
 waypointList.push_back(nextWayPoint);

 nextWayPoint.x = 0;
 nextWayPoint.y = r+d;
 nextWayPoint.z = z;
 nextWayPoint.psi = -180;
 waypointList.push_back(nextWayPoint);


 nextWayPoint.x = 0;
 nextWayPoint.y = 0;
 nextWayPoint.z = z;
 nextWayPoint.psi = 0;
 waypointList.push_back(nextWayPoint);

    rclcpp::Rate rate(3.0);
    int counter = 0;

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();


        if (check_waypoint_reached(.3) == 1) {
            if (counter < waypointList.size()) {
                set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
                counter++;
            } else {
                land();
            }
        }
    }

    rclcpp::spin(node);
    rclcpp::shutdown();



    return 0;
}
