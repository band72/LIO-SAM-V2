#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <cfloat>

using namespace std::chrono_literals;

class TargetDetector : public rclcpp::Node
{
public:
    TargetDetector(const rclcpp::NodeOptions & options)
    : Node("target_detector", options)
    {
        // ROS2 Parameters for Target Dimensions
        // We convert your inch measurements to meters with a generous tolerance,
        // because LiDAR will rarely hit every single millimeter of a 0.25" pole.
        this->declare_parameter<double>("intensity_threshold", 100.0); // Adjust based on your sensor
        this->declare_parameter<double>("target_min_height", 0.40); // ~16 inches minimum visible height
        this->declare_parameter<double>("target_max_height", 0.80); // ~31 inches max height
        this->declare_parameter<double>("target_max_width", 0.15); // max 6 inch width/depth

        this->get_parameter("intensity_threshold", intensity_threshold_);
        this->get_parameter("target_min_height", target_min_height_);
        this->get_parameter("target_max_height", target_max_height_);
        this->get_parameter("target_max_width", target_max_width_);

        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points_raw", 10,
            std::bind(&TargetDetector::cloudHandler, this, std::placeholders::_1));

        pub_landmark_ = this->create_publisher<geometry_msgs::msg::PointStamped>("lio_sam/landmark", 10);
        
        RCLCPP_INFO(this->get_logger(), "Target Detector Node Initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_landmark_;

    double intensity_threshold_;
    double target_min_height_;
    double target_max_height_;
    double target_max_width_;

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud_in);

        if (cloud_in->empty()) return;

        // 1. Intensity Filter: Keep only highly reflective points
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud_in);
        pass.setFilterFieldName("intensity");
        pass.setFilterLimits(intensity_threshold_, FLT_MAX);
        pass.filter(*cloud_filtered);

        if (cloud_filtered->empty()) return;

        // 2. Euclidean Clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud_filtered);

        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.15); // 15cm tolerance between points in a cluster
        ec.setMinClusterSize(5);      // Need at least 5 points to form a target
        ec.setMaxClusterSize(500);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        // 3. Analyze Clusters for Shape/Size
        for (const auto& cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            for (const auto& idx : cluster.indices)
                target_cloud->push_back((*cloud_filtered)[idx]);

            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(*target_cloud, min_pt, max_pt);

            double width_x = max_pt.x - min_pt.x;
            double width_y = max_pt.y - min_pt.y;
            double height_z = max_pt.z - min_pt.z;

            // Check if bounding box matches our parameters
            if (height_z >= target_min_height_ && height_z <= target_max_height_ &&
                width_x <= target_max_width_ && width_y <= target_max_width_)
            {
                // We found a target! Calculate centroid
                double cx = (max_pt.x + min_pt.x) / 2.0;
                double cy = (max_pt.y + min_pt.y) / 2.0;
                double cz = (max_pt.z + min_pt.z) / 2.0;

                // Publish Landmark
                geometry_msgs::msg::PointStamped landmark_msg;
                landmark_msg.header = msg->header; // Keep original scan timestamp!
                landmark_msg.point.x = cx;
                landmark_msg.point.y = cy;
                landmark_msg.point.z = cz;

                pub_landmark_->publish(landmark_msg);
                
                RCLCPP_INFO(this->get_logger(), 
                    "Target detected! height: %.2fm, width: %.2fm. Publishing landmark at (%.2f, %.2f, %.2f)", 
                    height_z, std::max(width_x, width_y), cx, cy, cz);
            }
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<TargetDetector>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
