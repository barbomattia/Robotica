
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h" // Include il messaggio per la PointCloud2

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Funzione di callback chiamata ogni volta che arriva un nuovo messaggio sulla point_cloud

    // Qui puoi scrivere la logica per manipolare i dati della point cloud
    // Nel caso di questo esempio, stampiamo solo il numero di punti nella nuvola
    ROS_INFO("Nuova nuvola di punti ricevuta. Numero di punti: %d", msg->width * msg->height);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "point_cloud_subscriber"); // Inizializza il nodo
    ros::NodeHandle n; // Crea un handle per il nodo

    // Crea un subscriber per il topic della point cloud
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/ur5/zed_node/point_cloud/cloud_registered", 10, pointCloudCallback);

    // Loop di esecuzione per la ricezione dei messaggi
    ros::spin();

    return 0;
}