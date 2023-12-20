#include "ros/ros.h"
#include "motion_planner/AddTwoInts.h"


bool add(motion_planner::AddTwoInts::Request &req, motion_planner::AddTwoInts::Response &res)
{
    res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    ros::spin();
    return 0;
} 