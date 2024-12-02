#include "pointcloud_rotation_node.h"

pointcloud_rotation_node::pointcloud_rotation_node(/* args */) : Node("pointcloud_rotation_node")
{

    // ==================  variables for ground remove  ==================
    this->declare_parameter("num_seg_", 50);
    this->declare_parameter("num_iter_", 25);
    this->declare_parameter("num_lpr_", 10);
    this->declare_parameter("th_seeds_", 1.0);
    this->declare_parameter("th_dist_", 0.3);
    this->declare_parameter("sensor_height_", 1.73);

    this->get_parameter("num_seg_", num_seg_);
    this->get_parameter("num_iter_", num_iter_);
    this->get_parameter("num_lpr_", num_lpr_);
    this->get_parameter("th_seeds_", th_seeds_);
    this->get_parameter("th_dist_", th_dist_);
    this->get_parameter("sensor_height_", sensor_height_);

    // ==================  variables for pointcloud voxel  ==================
    this->declare_parameter("voxel_leaf_size_x", double(0.0));
    this->declare_parameter("voxel_leaf_size_y", double(0.0));
    this->declare_parameter("voxel_leaf_size_z", double(0.0));
    this->declare_parameter("voxel_condition", false);

    this->get_parameter("voxel_leaf_size_x", voxel_leaf_size_x_);
    this->get_parameter("voxel_leaf_size_y", voxel_leaf_size_y_);
    this->get_parameter("voxel_leaf_size_z", voxel_leaf_size_z_);

    this->get_parameter("voxel_condition", voxel_condition);

    // ==================  variables for ROI boundaries  ==================

    this->declare_parameter("roi_max_x_", double(0.0));
    this->declare_parameter("roi_max_y_", double(0.0));
    this->declare_parameter("roi_max_z_", double(0.0));

    this->declare_parameter("roi_min_x_", double(0.0));
    this->declare_parameter("roi_min_y_", double(0.0));
    this->declare_parameter("roi_min_z_", double(0.0));

    this->get_parameter("roi_max_x_", roi_max_x_);
    this->get_parameter("roi_max_y_", roi_max_y_);
    this->get_parameter("roi_max_z_", roi_max_z_);

    this->get_parameter("roi_min_x_", roi_min_x_);
    this->get_parameter("roi_min_y_", roi_min_y_);
    this->get_parameter("roi_min_z_", roi_min_z_);

    // ==================  variables for pointcloud rotation  ==================

    // ==================  variables for ground remove  ==================
    this->declare_parameter("num_seg_", 50);
    this->declare_parameter("num_iter_", 25);
    this->declare_parameter("num_lpr_", 10);
    this->declare_parameter("th_seeds_", 1.0);
    this->declare_parameter("th_dist_", 0.3);
    this->declare_parameter("sensor_height_", 1.73);

    this->get_parameter("num_seg_", num_seg_);
    this->get_parameter("num_iter_", num_iter_);
    this->get_parameter("num_lpr_", num_lpr_);
    this->get_parameter("th_seeds_", th_seeds_);
    this->get_parameter("th_dist_", th_dist_);
    this->get_parameter("sensor_height_", sensor_height_);

    // ==================  variables for pointcloud voxel  ==================
    this->declare_parameter("voxel_leaf_size_x", double(0.0));
    this->declare_parameter("voxel_leaf_size_y", double(0.0));
    this->declare_parameter("voxel_leaf_size_z", double(0.0));
    this->declare_parameter("voxel_condition", false);

    this->get_parameter("voxel_leaf_size_x", voxel_leaf_size_x_);
    this->get_parameter("voxel_leaf_size_y", voxel_leaf_size_y_);
    this->get_parameter("voxel_leaf_size_z", voxel_leaf_size_z_);

    this->get_parameter("voxel_condition", voxel_condition);

    // ==================  variables for ROI boundaries  ==================

    this->declare_parameter("roi_max_x_", double(0.0));
    this->declare_parameter("roi_max_y_", double(0.0));
    this->declare_parameter("roi_max_z_", double(0.0));

    this->declare_parameter("roi_min_x_", double(0.0));
    this->declare_parameter("roi_min_y_", double(0.0));
    this->declare_parameter("roi_min_z_", double(0.0));

    this->get_parameter("roi_max_x_", roi_max_x_);
    this->get_parameter("roi_max_y_", roi_max_y_);
    this->get_parameter("roi_max_z_", roi_max_z_);

    this->get_parameter("roi_min_x_", roi_min_x_);
    this->get_parameter("roi_min_y_", roi_min_y_);
    this->get_parameter("roi_min_z_", roi_min_z_);

    // ==================  variables for pointcloud rotation  ==================
    this->declare_parameter("sensor_rotation_y_", 0.0);
    this->get_parameter("sensor_rotation_y_", sensor_rotation_y_);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10, std::bind(&pointcloud_rotation_node::pointCloudCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_rotated", 10);
    voxel_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_removal", 10);

    ROI_MAX_POINT = Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1);
    ROI_MIN_POINT = Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1);

    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->voxel_leaf_size_x: %f \033[0m", voxel_leaf_size_x_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->voxel_leaf_size_y: %f \033[0m", voxel_leaf_size_y_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->voxel_leaf_size_z: %f \033[0m", voxel_leaf_size_z_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->voxel_condition: %d \033[0m", voxel_condition);

    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_max_x: %f \033[0m", roi_max_x_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_max_y: %f \033[0m", roi_max_y_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_max_z: %f \033[0m", roi_max_z_);

    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_min_x: %f \033[0m", roi_min_x_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_min_y: %f \033[0m", roi_min_y_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_min_z: %f \033[0m", roi_min_z_);

    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> num_seg: %d \033[0m", num_seg_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> num_iter: %d \033[0m", num_iter_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> num_lpr: %d \033[0m", num_lpr_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> th_seeds: %f \033[0m", th_seeds_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> th_dist: %f \033[0m", th_dist_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> sensor_height: %f \033[0m", sensor_height_);

    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> sensor_rotation_y: %f \033[0m", sensor_rotation_y_);
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> pointcloud_rotation_node initialized.\033[0m");
}

pointcloud_rotation_node::~pointcloud_rotation_node()
{
}

void pointcloud_rotation_node::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    auto init_time = std::chrono::system_clock::now();


    auto init_time = std::chrono::system_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *input_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*input_cloud, *transformed_cloud, rotation_matrix_);

    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*transformed_cloud, ground_msg);
    ground_msg.header.frame_id = msg->header.frame_id;
    ground_msg.header.stamp = msg->header.stamp;
    pub_->publish(ground_msg);

    // Apply ROI filtering
    pcl::CropBox<pcl::PointXYZI> roi_filter;
    roi_filter.setInputCloud(transformed_cloud);
    roi_filter.setMax(ROI_MAX_POINT);
    roi_filter.setMin(ROI_MIN_POINT);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>());
    roi_filter.filter(*cloud_roi);

    if (voxel_condition)
    {
        // create voxel grid object
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud_roi);
        vg.setLeafSize(voxel_leaf_size_x_, voxel_leaf_size_y_, voxel_leaf_size_z_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        vg.filter(*filtered_cloud);

        // Separate ground and non-ground points
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points(new pcl::PointCloud<pcl::PointXYZI>());
        extractInitialSeeds(filtered_cloud, seed_points);

        Model model = estimatePlane(*seed_points);

        for (auto &point : filtered_cloud->points)
        {
            float dist = model.normal(0) * point.x + model.normal(1) * point.y + model.normal(2) * point.z + model.d;
            if (dist < th_dist_)
            {
                ground_points->points.push_back(point);
            }
            else
            {
                notground_points->points.push_back(point);
            }
        }

        // convert back to ROS datatype
        sensor_msgs::msg::PointCloud2 ground_msg;
        pcl::toROSMsg(*notground_points, ground_msg);
        ground_msg.header = msg->header;
        voxel_grid_pub_->publish(ground_msg);
    }
    else
    {
        sensor_msgs::msg::PointCloud2 downsampled_cloud_msg;
        pcl::toROSMsg(*cloud_roi, downsampled_cloud_msg);
        downsampled_cloud_msg.header = msg->header;
        voxel_grid_pub_->publish(downsampled_cloud_msg);
    }

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    std::cout << blue << "Execution time for path creation: " << duration << " ms" << reset << std::endl;
}

pointcloud_rotation_node::Model pointcloud_rotation_node::estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &seed_points)
{
    Eigen::Matrix3f cov_matrix;
    Eigen::Vector4f centroid;
    pcl::computeMeanAndCovarianceMatrix(seed_points, cov_matrix, centroid);

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_matrix, Eigen::ComputeFullU);
    Model model;
    model.normal = svd.matrixU().col(2);
    model.d = -(model.normal.transpose() * centroid.head<3>())(0, 0);

    return model;
}

void pointcloud_rotation_node::extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &seed_points)
{
    std::vector<pcl::PointXYZI> cloud_sorted(cloud_in->points.begin(), cloud_in->points.end());
    std::sort(cloud_sorted.begin(), cloud_sorted.end(), [](const pcl::PointXYZI &a, const pcl::PointXYZI &b)
              { return a.z < b.z; });

    auto it = std::find_if(cloud_sorted.begin(), cloud_sorted.end(), [&](const pcl::PointXYZI &point)
                           { return point.z >= -1.5 * sensor_height_; });

    cloud_sorted.erase(cloud_sorted.begin(), it);

    double LPR_height = 0;
    for (int i = 0; i < num_lpr_ && i < static_cast<int>(cloud_sorted.size()); ++i)
    {
        LPR_height += cloud_sorted[i].z;
    }
    LPR_height /= std::max(1, num_lpr_);

    seed_points->points.clear();
    for (const auto &point : cloud_sorted)
    {
        if (point.z < LPR_height + th_seeds_)
        {
            seed_points->points.push_back(point);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_rotation_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}