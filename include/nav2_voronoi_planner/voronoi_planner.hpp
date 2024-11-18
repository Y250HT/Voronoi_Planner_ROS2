#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>


#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "dynamicvoronoi/dynamicvoronoi.h"
#include "nav2_core/global_planner.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
/////////////////
//#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
/////////////////
#include "voronoi.hpp"

namespace nav2_voronoi_planner
{
class VoronoiPlanner : public nav2_core::GlobalPlanner, public rclcpp::Node
{
public:
    VoronoiPlanner() : Node("voronoi_node") { 
        /*start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&VoronoiPlanner::SetStart, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 10, std::bind(&VoronoiPlanner::SetGoal, this, std::placeholders::_1));*/

    }
    ~VoronoiPlanner() {
        RCLCPP_INFO(
            logger_, "Destroying plugin %s of type VoronoiPlanner",
            name_.c_str());
    }

    // plugin configure
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // plugin cleanup
    void cleanup() override;

    // plugin activate
    void activate() override;

    // plugin deactivate
    void deactivate() override;

    // This method creates path for given start and goal pose.
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;

protected:
    /*void SetStart(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start) {
        std::lock_guard<std::mutex> lock(start_mutex_);
        RCLCPP_INFO(this->get_logger(), "A new start is received.");
        start_.header = start->header;
        start_.pose = start->pose.pose;
        start_received_ = true;
        MakePlan();
    }


    void SetGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        RCLCPP_INFO(this->get_logger(), "A new goal is received.");
        goal_ = *goal;
        goal_received_ = true;
        MakePlan();
    }

    void MakePlan() {
        // Implementation for planning based on start and goal.
        if (start_received_ && goal_received_) {
            RCLCPP_INFO(this->get_logger(), "Planning from start to goal.");
            // Planning logic here.
        }
    }*/

    bool makePlan(const geometry_msgs::msg::PoseStamped& start,
                  const geometry_msgs::msg::PoseStamped& goal,
                  nav_msgs::msg::Path& plan);
    bool UpdateCostmap(nav2_costmap_2d::Costmap2DROS* costmap_ros);
    static void GetStartAndEndConfigurations(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal, double resolution,
        double origin_x, double origin_y, int* start_x, int* start_y, int* end_x,
        int* end_y);
    std::vector<std::vector<VoronoiData>> GetVoronoiDiagram(unsigned int size_x,
                                                            unsigned int size_y,
                                                            double resolution);

    static void PopulateVoronoiPath(
        const std::vector<std::pair<int, int>>& searched_result,
        const std_msgs::msg::Header& header, double resolution, double origin_x,
        double origin_y, nav_msgs::msg::Path & plan);

    void PublishVoronoiGrid(const DynamicVoronoi& voronoi,
                            const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& pub);

    void PublishVoronoiPath(
        nav_msgs::msg::Path& plan,
        const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub);
private:
    std::unique_ptr<Voronoi> voronoi_planner_ = nullptr;
    //const nav2_costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
    nav2_costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
    nav2_costmap_2d::Costmap2D * costmap_2d_ = nullptr;
    nav2_costmap_2d::LayeredCostmap* layered_costmap_ = nullptr;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    std::string global_frame_, name_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("VoronoiPlanner")};
    bool initialized_ = false;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr voronoi_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    /*rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    geometry_msgs::msg::PoseStamped start_;
    geometry_msgs::msg::PoseStamped goal_;
    bool start_received_ = false;
    bool goal_received_  = false;

    std::mutex start_mutex_;
    std::mutex goal_mutex_;*/

};

}
