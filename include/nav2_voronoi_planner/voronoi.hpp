#pragma once
#include <iostream>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include <vector>

#include "nav2_voronoi_planner/constants.hpp"
#include "nav2_voronoi_planner/heap.hpp"
#include "nav2_voronoi_planner/node2d.hpp"

namespace nav2_voronoi_planner 
{
struct VoronoiData {
  bool is_voronoi = false;
  double dist = 0.0;
};

struct GridSearchPrimitives {
  std::array<int, kNumOfGridSearchActions> dx;
  std::array<int, kNumOfGridSearchActions> dy;
  std::array<int, kNumOfGridSearchActions> dxy_cost;
};

struct GridSearchResult {
  std::vector<std::pair<int, int>> grid_path;
  int path_cost = 0;
};

class Voronoi
{
public:
    enum class SearchType { A_STAR, DP };
    Voronoi() = default;
    
    ~Voronoi();
    void Init(int max_grid_x, int max_grid_y, double circumscribed_radius);
    bool Search(int sx, int sy, int ex, int ey,
                std::vector<std::vector<VoronoiData>>&& gvd_map,
                std::vector<std::pair<int, int>>* path);
private:
    bool SetStart(int start_x, int start_y);
    bool SetEnd(int end_x, int end_y);
    Node2d* GetNode(int grid_x, int grid_y);
    bool IsValidCell(int grid_x, int grid_y) const;
    // 添加起始点终点判断
    bool IsValidCellStartAndEnd(int grid_x, int grid_y) const;


    int CalcHeuCost(int grid_x, int grid_y) const;
    bool SetStartAndEndConfiguration(int sx, int sy, int ex, int ey);
    bool IsWithinMap(int grid_x, int grid_y) const;
    bool CheckVoronoi(int grid_x, int grid_y) const;
    int CalcGridXYIndex(int grid_x, int grid_y) const;
    int GetKey(const Node2d* node) const;
    void UpdateSuccs(const Node2d* curr_node);
    int GetActionCost(int curr_x, int curr_y, int action_id) const;
    void LoadGridSearchResult(int end_x, int end_y,
                            GridSearchResult* result) const;
    void ComputeGridSearchActions();
    void Clear();
    bool SearchPathToVoronoiEdges(int sx, int sy, int ex, int ey,
                                int* voronoi_goal_x, int* voronoi_goal_y,
                                GridSearchResult* result);
    bool SearchPathAlongVoronoiEdges(int sx, int sy, int ex, int ey,
                                   GridSearchResult* result);

    int max_grid_x_ = 0; // 0
    int max_grid_y_ = 0; // 0
    std::vector<std::vector<Node2d>> dp_lookup_table_;

    std::unique_ptr<Heap> open_list_ = nullptr;
    
    std::vector<Node::NodeStatus> closed_list_;
    
    double circumscribed_radius_;  // = 0.23142
    Node2d* start_node_ = nullptr;
    Node2d* end_node_ = nullptr;
    std::vector<std::vector<VoronoiData>> gvd_map_;

    GridSearchPrimitives actions_;
    SearchType search_type_;
    bool need_check_voronoi_ = false;
    std::size_t iterations_ = 0U;
    bool initialized_ = false;
};

} // namespace nav2_voronoi_planner 