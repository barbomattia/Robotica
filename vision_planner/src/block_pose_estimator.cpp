#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vision_planner/BlockPose.h>
#include <open3d/Open3D.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

bool isBlock(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const double& xmin, const double& ymin, const double& xmax, const double& ymax, const double& depth_threshold)
{
    // Iterate over the points in the bounding box
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;
        double z = cloud->points[i].z;

        // Check if the point is within the bounding box
        if (x >= xmin && x <= xmax && y >= ymin && y <= ymax)
        {
            // Check depth differences with neighboring points
            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    if (dx == 0 && dy == 0)
                        continue;  // Skip the central point

                    double neighbor_x = x + dx * 0.01;  // Adjust the step size as needed
                    double neighbor_y = y + dy * 0.01;  // Adjust the step size as needed

                    size_t neighbor_index = findNearestPointIndex(cloud, neighbor_x, neighbor_y);

                    if (neighbor_index != pcl::PointCloud<pcl::PointXYZ>::npos)
                    {
                        double neighbor_z = cloud->points[neighbor_index].z;

                        // Check if the depth difference exceeds the threshold
                        if (std::abs(z - neighbor_z) > depth_threshold)
                        {
                            // This region likely corresponds to a block
                            return true;
                        }
                    }
                }
            }
        }
    }

    // No significant depth differences found, likely the background
    return false;
}

size_t findNearestPointIndex(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const double& target_x, const double& target_y)
{
    double min_distance = std::numeric_limits<double>::max();
    size_t nearest_index = pcl::PointCloud<pcl::PointXYZ>::npos;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;

        double distance = std::sqrt((x - target_x) * (x - target_x) + (y - target_y) * (y - target_y));

        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_index = i;
        }
    }

    return nearest_index;
}

class BlockPoseEstimator
{
public:
    BlockPoseEstimator(ros::NodeHandle nh)
        : nh_(nh)
    {
        // Subscribe to the point cloud topic
        pointcloud_sub_ = nh_.subscribe("/zed2_camera/point_cloud", 1, &BlockPoseEstimator::pointcloudCallback, this);

        // Publish block pose
        block_pose_pub_ = nh_.advertise<vision_planner::BlockPose>("/block_pose", 1);

        // Load the full 3D model (replace this with your actual 3D model loading)
        open3d::io::ReadPointCloud("path/to/your/full_model.pcd", full_model_point_cloud_);

        // Set the bounding box coordinates (replace with actual values from YOLO)
        xmin_ = 0.1;
        ymin_ = 0.2;
        xmax_ = 0.4;
        ymax_ = 0.6;
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
    {
        // Convert ROS point cloud message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*pc_msg, cloud);

        // Convert PCL point cloud to Open3D point cloud
        auto point_cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
        point_cloud_ptr->points_.resize(cloud.points.size());
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            point_cloud_ptr->points_[i](0) = cloud.points[i].x;
            point_cloud_ptr->points_[i](1) = cloud.points[i].y;
            point_cloud_ptr->points_[i](2) = cloud.points[i].z;
        }

        // Convert 2D bounding box to 3D bounding box using the full model
        open3d::geometry::AxisAlignedBoundingBox bbox_3d;
        bbox_3d.CreateFromPoints(full_model_point_cloud_, {xmin_, ymin_, -1.0}, {xmax_, ymax_, 1.0});

        // Get the points within the 3D bounding box from the full model
        auto model_cropped_point_cloud_ptr = full_model_point_cloud_.Crop(bbox_3d);

        // Check if it's actually a block or the background
        if (!isBlock(point_cloud_ptr, xmin_, ymin_, xmax_, ymax_, 0.01))
        {
            ROS_WARN("No block found!");
            return;
        }

        // Create a KDTree for efficient nearest neighbor search in the scene
        open3d::geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(*point_cloud_ptr);

        // Find correspondences between keypoints in the cropped model and keypoints in the scene
        open3d::pipelines::registration::RegistrationResult registration_result =
            open3d::pipelines::registration::RegistrationColoredICP(
                *model_cropped_point_cloud_ptr, *point_cloud_ptr, 0.02, Eigen::Matrix4d::Identity());

        // Get the transformation matrix (pose) from the registration result
        auto transformation = registration_result.transformation_;

        // Publish block pose
        vision_planner::BlockPose block_pose_msg;
        // Fill block_pose_msg fields with your data
        // e.g., block_pose_msg.pose = transformation;

        block_pose_pub_.publish(block_pose_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher block_pose_pub_;
    open3d::geometry::PointCloud full_model_point_cloud_;
    double xmin_, ymin_, xmax_, ymax_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "block_pose_estimator_node");

    ros::NodeHandle nh;

    BlockPoseEstimator block_pose_estimator(nh);

    ros::spin();

    return 0;
}
