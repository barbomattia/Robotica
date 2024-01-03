#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Open3D/Open3D.h>


// Add any other necessary headers for your vision processing

class VisionPlannerNode
{
public:
    VisionPlannerNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
        : nh_(nh), private_nh_(private_nh)
    {
        // Initialize ROS publishers and subscribers
        image_sub_ = nh_.subscribe("/zed2_camera/image_raw", 1, &VisionPlannerNode::imageCallback, this);
        pointcloud_sub_ = nh_.subscribe("/zed2_camera/point_cloud", 1, &VisionPlannerNode::pointcloudCallback, this);
        block_pose_pub_ = nh_.advertise<your_package_msgs::BlockPose>("/block_poses", 1);

        // Load parameters from the parameter server
        private_nh_.param<std::string>("yolo_model_path", yolo_model_path_, "/default/path/to/yolo_model.weights");
        private_nh_.param<double>("yolo_confidence_threshold", yolo_confidence_threshold_, 0.5);
        private_nh_.param<double>("camera_fx", camera_fx_, 525.0);
        private_nh_.param<double>("camera_fy", camera_fy_, 525.0);
        private_nh_.param<double>("camera_cx", camera_cx_, 319.5);
        private_nh_.param<double>("camera_cy", camera_cy_, 239.5);

        // Initialize any other components or libraries
        // e.g., YOLO detector, block pose estimator, etc.
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
    {
        // Convert ROS image message to OpenCV image
        cv::Mat image;
        try
        {
            image = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("CV Bridge Exception: %s", e.what());
            return;
        }

        // Perform image processing with OpenCV and YOLO
        // ...

        // Assuming you have block poses as a result
        your_package_msgs::BlockPose block_pose;
        // Fill block_pose message fields with your data

        // Publish block pose
        block_pose_pub_.publish(block_pose);
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
    {
        // Convert ROS point cloud message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*pc_msg, cloud);

        // Perform point cloud processing
        // ...

        // Assuming you have block poses as a result
        your_package_msgs::BlockPose block_pose;
        // Fill block_pose message fields with your data

        // Publish block pose
        block_pose_pub_.publish(block_pose);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher block_pose_pub_;

    // Parameters
    std::string yolo_model_path_;
    double yolo_confidence_threshold_;
    double camera_fx_, camera_fy_, camera_cx_, camera_cy_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_planner_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    VisionPlannerNode vision_planner_node(nh, private_nh);

    ros::spin();

    return 0;
}
