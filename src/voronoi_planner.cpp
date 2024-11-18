#include "nav2_voronoi_planner/voronoi_planner.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/voronoi_layer.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_voronoi_planner/util.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>


//#include "tf2_ros/buffer.h"
//#include "tf2_ros/transform_listener.h"
// 打印调试信息
// std::cout << "" << std::endl;
// std::cout << "\033[" <<  << "\033[0m" << std::endl;

namespace nav2_voronoi_planner 
{
void VoronoiPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    tf_ = tf;
    name_ = name;
    costmap_2d_ = costmap_ros->getCostmap();
  
    global_frame_ = costmap_ros->getGlobalFrameID();
    node_ = parent.lock();

    clock_ = node_->get_clock();
    logger_ = node_->get_logger();
    
    if (!UpdateCostmap(costmap_ros.get())) {
        std::cout << "\033[" << "Failed to update costmap." << "\033[0m" << std::endl;
        return;
    }
    std::cout << static_cast<int>(costmap_2d_->getSizeInCellsX()) << std::endl;
    std::cout << static_cast<int>(costmap_2d_->getSizeInCellsX()) << std::endl;
    std::cout << static_cast<int>(costmap_2d_->getSizeInCellsX()) << std::endl;

    std::cout << static_cast<int>(costmap_2d_->getSizeInCellsY()) << std::endl;
    std::cout << static_cast<int>(costmap_2d_->getSizeInCellsY()) << std::endl;
    std::cout << static_cast<int>(costmap_2d_->getSizeInCellsY()) << std::endl;

    std::cout << layered_costmap_->getCircumscribedRadius() << std::endl;
    std::cout << layered_costmap_->getCircumscribedRadius() << std::endl;
    std::cout << layered_costmap_->getCircumscribedRadius() << std::endl;

    

    voronoi_planner_ = std::make_unique<Voronoi>();
  
    voronoi_planner_->Init(static_cast<int>(costmap_2d_->getSizeInCellsX()),
                           static_cast<int>(costmap_2d_->getSizeInCellsY()),
                           layered_costmap_->getCircumscribedRadius());

    initialized_ = true;

    voronoi_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("voronoi_grid", 1);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("voronoi_path", 1);
    RCLCPP_INFO(
        logger_, "Configuring plugin %s of type VoronoiPlanner",
        name_.c_str());
}

void VoronoiPlanner::cleanup()
{
    RCLCPP_INFO(
        logger_, "Cleaning up plugin %s of type VoronoiPlanner",
        name_.c_str());
    voronoi_planner_.reset();
}

void VoronoiPlanner::activate()
{
    RCLCPP_INFO(
        logger_, "Activating plugin %s of type VoronoiPlanner",
        name_.c_str());
    // auto node = node_.lock();
    auto node_weak = std::weak_ptr<rclcpp_lifecycle::LifecycleNode>(node_);
    auto node = node_weak.lock();
}

void VoronoiPlanner::deactivate()
{
    RCLCPP_INFO(
        logger_, "Deactivating plugin %s of type VoronoiPlanner",
        name_.c_str());
}

nav_msgs::msg::Path VoronoiPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) 
{
    nav_msgs::msg::Path global_path;
    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_) {
        RCLCPP_ERROR(
        node_->get_logger(), "Planner will only except start position from %s frame",
        global_frame_.c_str());
        return global_path;
    }

    if (goal.header.frame_id != global_frame_) {
        RCLCPP_INFO(
        node_->get_logger(), "Planner will only except goal position from %s frame",
        global_frame_.c_str());
        return global_path;
    }

    global_path.poses.clear();

    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;

    if (!makePlan(start, goal, global_path)) {
        std::cout << "failed to create plan" << std::endl;
    }
    return global_path;
}

bool VoronoiPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                              const geometry_msgs::msg::PoseStamped& goal,
                              nav_msgs::msg::Path & plan)
{
    // Check if VoronoiPlannerROS has been initialized.
    if (!initialized_) {
        // LOG(ERROR) << "VoronoiPlannerROS has not been initialized.";
        std::cout << "VoronoiPlannerROS has not been initialized." << std::endl;
        return false;
    }
    plan.poses.clear();
    costmap_ros_->updateMapWithVoronoi();

    costmap_ros_->resetLayers(); // 防止残影影响全局规划

    costmap_ros_->updateMapWithVoronoi();
    // Get start and end configurations.
    int start_x = 0;
    int start_y = 0;
    int end_x = 0;
    int end_y = 0;
    GetStartAndEndConfigurations(
        start, goal, costmap_2d_->getResolution(), costmap_2d_->getOriginX(),
        costmap_2d_->getOriginY(), &start_x, &start_y, &end_x, &end_y);
    std::cout << start_x << std::endl;
    std::cout << start_x << std::endl;
    std::cout << start_x << std::endl;
    std::cout << start_x << std::endl;
    std::cout << start_y << std::endl;
    std::cout << start_y << std::endl;
    std::cout << start_y << std::endl;
    std::cout << start_y << std::endl;
    std::cout << start_y << std::endl;

    // Get Voronoi diagram.
    std::vector<std::vector<VoronoiData>> gvd_map = GetVoronoiDiagram(
        costmap_2d_->getSizeInCellsX(), costmap_2d_->getSizeInCellsY(),
        costmap_2d_->getResolution());

    

    if (gvd_map.empty()) {
        //LOG(ERROR) << "Voronoi layer is not available.";
        RCLCPP_WARN(logger_, "Voronoi layer is not available.");
        return false;
    }

    // Search path via Voronoi planner.
    std::vector<std::pair<int, int>> path;
    if (voronoi_planner_->Search(start_x, start_y, end_x, end_y,
                                    std::move(gvd_map), &path)) {
        RCLCPP_WARN(logger_, "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    } else {
        // LOG(ERROR) << "Failed to find the shortest Voronoi path";
        RCLCPP_WARN(logger_, "Failed to find the shortest Voronoi path");
        return false;
    }

    // Populate global path.
    PopulateVoronoiPath(path, start.header, costmap_2d_->getResolution(),
                        costmap_2d_->getOriginX(), costmap_2d_->getOriginY(),
                        plan);
    plan.poses.push_back(goal);

    //添加标志pose
    geometry_msgs::msg::PoseStamped mark_pose;
    mark_pose.header.frame_id = "map";
    mark_pose.pose = plan.poses.back().pose;
    plan.poses.push_back(mark_pose);

    // Publish Voronoi path.
    PublishVoronoiPath(plan, path_pub_);


    return true;
}

bool VoronoiPlanner::UpdateCostmap(nav2_costmap_2d::Costmap2DROS* costmap_ros)
{
    if (costmap_ros == nullptr) {
        // LOG(ERROR) << "costmap_ros == nullptr";
        RCLCPP_WARN(logger_, "costmap_ros == nullptr");
        return false;
    }

    costmap_ros_ = costmap_ros;
    costmap_2d_ = costmap_ros->getCostmap();
    layered_costmap_ = costmap_ros->getLayeredCostmap();
    if (costmap_2d_ == nullptr || layered_costmap_ == nullptr) {
        // LOG(ERROR) << "costmap_2d_ == nullptr || layered_costmap_ == nullptr";
        RCLCPP_WARN(logger_, "costmap_2d_ == nullptr || layered_costmap_ == nullptr");
        return false;
    }
    return true;
}

void VoronoiPlanner::GetStartAndEndConfigurations(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal, double resolution,
    double origin_x, double origin_y, int* start_x, int* start_y, int* end_x,
    int* end_y) 
{
    // Start configuration.
    *start_x = ContXY2Disc(start.pose.position.x - origin_x, resolution);
    *start_y = ContXY2Disc(start.pose.position.y - origin_y, resolution);

    // End configuration.
    *end_x = ContXY2Disc(goal.pose.position.x - origin_x, resolution);
    *end_y = ContXY2Disc(goal.pose.position.y - origin_y, resolution);
}
// 获取维诺图
std::vector<std::vector<VoronoiData>> VoronoiPlanner::GetVoronoiDiagram(unsigned int size_x,
                                                                 unsigned int size_y,
                                                                 double resolution)
{
    //std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins =
    //  layered_costmap_->getPlugins();
    std::vector<std::shared_ptr<nav2_costmap_2d::Layer>>* plugins =
        layered_costmap_->getPlugins();
    if (plugins == nullptr) {
        // LOG(ERROR) << "plugins == nullptr";
        std::cout << "plugins == nullptr" << std::endl;
        return std::vector<std::vector<VoronoiData>>();
    }

    // Check if costmap has a Voronoi layer.
    for (auto& plugin : *plugins) {
        //auto voronoi_layer =
        //    boost::dynamic_pointer_cast<costmap_2d::VoronoiLayer>(plugin);
        auto voronoi_layer = std::dynamic_pointer_cast<nav2_costmap_2d::VoronoiLayer>(plugin);
        if (voronoi_layer == nullptr) {
            continue;
        }

        std::lock_guard<std::mutex> lock(voronoi_layer->mutex());

        const DynamicVoronoi& voronoi = voronoi_layer->voronoi();
        PublishVoronoiGrid(voronoi, voronoi_pub_);
        std::vector<std::vector<VoronoiData>> gvd_map;
        gvd_map.resize(size_x);
        for (int i = 0; i < static_cast<int>(size_x); ++i) {
            gvd_map[i].resize(size_y);
            for (int j = 0; j < static_cast<int>(size_y); ++j) {
                gvd_map[i][j].dist = voronoi.getDistance(i, j) * resolution;
                gvd_map[i][j].is_voronoi = voronoi.isVoronoi(i, j);
            }
        }
        return gvd_map;
    }
    std::cout << "Failed to get a Voronoi layer for Voronoi planner" << std::endl;
    // LOG(ERROR) << "Failed to get a Voronoi layer for Voronoi planner";
    return std::vector<std::vector<VoronoiData>>();
}

void VoronoiPlanner::PopulateVoronoiPath(
    const std::vector<std::pair<int, int>>& searched_result,
    const std_msgs::msg::Header& header, double resolution, double origin_x,
    double origin_y, nav_msgs::msg::Path & plan)
{
    // Sanity checks.
    // CHECK_NOTNULL(plan);

    plan.poses.clear();
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = header;
    for (const auto& pose : searched_result) {
        pose_stamped.pose.position.x =
            DiscXY2Cont(pose.first, resolution) + origin_x;
        pose_stamped.pose.position.y =
            DiscXY2Cont(pose.second, resolution) + origin_y;
        //pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, 0.0); // 设置四元数的欧拉角为 (roll, pitch, yaw)
        pose_stamped.pose.orientation = tf2::toMsg(quaternion);
        plan.poses.push_back(pose_stamped);
    }
}

void VoronoiPlanner::PublishVoronoiGrid(const DynamicVoronoi& voronoi,
                                        const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& pub)
{
    // Publish whole grid.
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = clock_->now();
    grid.info.resolution = costmap_2d_->getResolution();

    grid.info.width = costmap_2d_->getSizeInCellsX();
    grid.info.height = costmap_2d_->getSizeInCellsY();

    grid.info.origin.position.x = costmap_2d_->getOriginX();
    grid.info.origin.position.y = costmap_2d_->getOriginY();
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(costmap_2d_->getSizeInCellsX() *
                     costmap_2d_->getSizeInCellsY());

    for (int x = 0; x < static_cast<int>(costmap_2d_->getSizeInCellsX()); ++x) {
        for (int y = 0; y < static_cast<int>(costmap_2d_->getSizeInCellsY()); ++y) {
        if (voronoi.isVoronoi(x, y)) {
            grid.data[x + y * costmap_2d_->getSizeInCellsX()] = static_cast<signed char>(128U);
        } else {
                grid.data[x + y * costmap_2d_->getSizeInCellsX()] = static_cast<signed char>(0U);
            }
        }
    }
    pub->publish(grid);
}

void VoronoiPlanner::PublishVoronoiPath(
        nav_msgs::msg::Path& plan,
        const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub)
{
    if (plan.poses.empty()) return;
    nav_msgs::msg::Path gui_path;
    gui_path.header = plan.header;
    gui_path.poses = plan.poses;
    pub->publish(gui_path);
}


}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_voronoi_planner::VoronoiPlanner, nav2_core::GlobalPlanner)
