#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>

/* JOINS ORDER 
1 -> primo joint ( spalla )
2 -> secondo joint ( spalla 2)
3 -> terzo joint ( gomito)
4 -> quarto joint ( polso 1 )
5 -> quinti joitn ( polso 2 )
6 -> sesto joitn ( polso 3 )
7 -> grip 1
8 -> grip 2

*/

int main(int argc, char **argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
    ros::Rate loop_rate(10);
    int count = 0;

    while (ros::ok()){
        std_msgs::Float64MultiArray jointPositions;
        jointPositions.data = { 4.35681, -1.78591, 2.25505, -2.04009, 1.5714, -2.23041, 0, 0}; // Posizioni desiderate dei joint in radianti
        
        pub.publish(jointPositions);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}