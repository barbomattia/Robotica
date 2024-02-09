#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"   // Include il messaggio per la PointCloud2
#include "sensor_msgs/Image.h"         // Include il messaggio per le Immagini


/**
 * @brief Callback function for handling incoming point cloud messages.
 * 
 * This function is called every time a new message is received on the 'pointCloud2' topic.
 * It logs information about the received point cloud, including the number of points it contains.
 * 
 * @param msg Pointer to the received point cloud message.
 */
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Funzione di callback chiamata ogni volta che arriva un nuovo messaggio sulla point_cloud
    ROS_INFO("Nuova nuvola di punti ricevuta. Numero di punti: %d", msg->width * msg->height);
}


/**
 * @brief Callback function for handling incoming image messages.
 * 
 * This function is called every time a new message is received on the 'Image' topic.
 * It logs information about the received image, including its dimensions.
 * 
 * @param msg Pointer to the received image message.
 */
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // Funzione di callback chiamata ogni volta che arriva un nuovo messaggio sulla point_cloud
    ROS_INFO("Nuova immagine ricevuta. Dimesnioni : %d, %d", msg->width , msg->height);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_subscriber"); // Inizializza il nodo
    ros::NodeHandle n; // Crea un handle per il nodo

    // Crea un subscriber per il topic della point cloud
    ros::Subscriber sub1 = n.subscribe<sensor_msgs::PointCloud2>("/ur5/zed_node/point_cloud/cloud_registered", 10, pointCloudCallback);

    // Crea un subscriber per il topic delle immagini 
    ros::Subscriber sub2 = n.subscribe<sensor_msgs::Image>("/ur5/zed_node/left/image_rect_color", 10, imageCallback);

    // Loop di esecuzione per la ricezione dei messaggi
    ros::spin();

    return 0;
}