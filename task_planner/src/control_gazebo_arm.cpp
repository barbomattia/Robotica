#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>

int main(int argc, char **argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
    ros::Rate loop_rate(10);
    int count = 0;

    while (ros::ok()){
        std_msgs::Float64MultiArray jointPositions;
        jointPositions.data = {1.8, -2, 2, 0, 0, 0, 1, 1}; // Posizioni desiderate dei joint in radianti
        
        pub.publish(jointPositions);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}