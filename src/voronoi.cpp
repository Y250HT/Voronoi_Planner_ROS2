#include "nav2_voronoi_planner/voronoi.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>  // NOLINT

// #include "glog/logging.h"

#include "nav2_voronoi_planner/heap.hpp"
// 打印调试信息
// std::cout << "" << std::endl;
// std::cout << "\033[" <<  << "\033[0m" << std::endl;
namespace nav2_voronoi_planner
{
void Voronoi::Init(int max_grid_x, int max_grid_y,
                   double circumscribed_radius) {
                    //initialized_ = true;
  if (initialized_) {
    std::cout << "VoronoiPlanner has been initialized." << std::endl;
    //LOG(INFO) << "VoronoiPlanner has been initialized.";
    return;
  }

  max_grid_x_ = max_grid_x;
  max_grid_y_ = max_grid_y;
  circumscribed_radius_ = circumscribed_radius;
  
  open_list_ = std::make_unique<Heap>();
  closed_list_.resize(max_grid_x_ * max_grid_y_,
                      Node::NodeStatus::OPEN);

  std::cout << closed_list_.size() << std::endl;

  // 初始化DP查找表
  dp_lookup_table_.resize(max_grid_x_);
  for (int grid_x = 0; grid_x < max_grid_x; ++grid_x) {
    for (int grid_y = 0; grid_y < max_grid_y; ++grid_y) {
      dp_lookup_table_[grid_x].emplace_back(grid_x, grid_y);
    }
  }
  // 计算网格搜索动作
  ComputeGridSearchActions();
  initialized_ = true;
  std::cout << "VoronoiPlanner is initialized successfully." << std::endl;
  // LOG(INFO) << "VoronoiPlanner is initialized successfully.";
  
}

Voronoi::~Voronoi() { open_list_->Clear(); }

void Voronoi::Clear() {
  // clean up heap elements in open list
  open_list_->Clear();

  // clear closed list
  closed_list_.clear();
  closed_list_.resize(max_grid_x_ * max_grid_y_,
                      Node::NodeStatus::OPEN);

  start_node_ = nullptr;
  end_node_ = nullptr;
}

bool Voronoi::IsWithinMap(const int grid_x, const int grid_y) const {
  std::cout << grid_x << std::endl;
  std::cout << max_grid_x_ << std::endl;
  std::cout << grid_y << std::endl;
  std::cout << max_grid_y_ << std::endl;

  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;
  std::cout << (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_) << std::endl;


  return (grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 && grid_y < max_grid_y_);
}

bool Voronoi::CheckVoronoi(int grid_x, int grid_y) const {
  if (!need_check_voronoi_) {
    return true;
  }
  return gvd_map_[grid_x][grid_y].is_voronoi;
}

int Voronoi::CalcGridXYIndex(const int grid_x, const int grid_y) const {
  // DCHECK(IsWithinMap(grid_x, grid_y));
  if (IsWithinMap(grid_x, grid_y)) {
    std::cout << "Is in Map" << std::endl;
  }
  std::cout << grid_x + grid_y * max_grid_x_ << std::endl;
  std::cout << grid_x + grid_y * max_grid_x_ << std::endl;
  std::cout << grid_x + grid_y * max_grid_x_ << std::endl;
  std::cout << grid_x + grid_y * max_grid_x_ << std::endl;
  std::cout << grid_x + grid_y * max_grid_x_ << std::endl;
  std::cout << grid_x + grid_y * max_grid_x_ << std::endl;

  return grid_x + grid_y * max_grid_x_;
}

int Voronoi::GetKey(const Node2d* node) const {
  // CHECK_NOTNULL(node);
  std::cout << "5555555555555555555555555555\n";
  return search_type_ == SearchType::A_STAR ? node->g() + node->h() : node->g();
}

bool Voronoi::SetStart(const int start_x, const int start_y) {
  if (!IsValidCellStartAndEnd(start_x, start_y)) {
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    return false;
  }
  std::cout << "888888888888888888888888888888888888888888\n";
  start_node_ = GetNode(start_x, start_y);
  std::cout << "888888888888888888888888888888888888888888\n";
  return true;
}

bool Voronoi::SetEnd(const int end_x, const int end_y) {
  if (!IsValidCellStartAndEnd(end_x, end_y)) {
    return false;
  }
  end_node_ = GetNode(end_x, end_y);
  return true;
}

Node2d* Voronoi::GetNode(const int grid_x, const int grid_y) {
  // DCHECK(IsWithinMap(grid_x, grid_y));
  std::cout << "888888888888888888888888888888888888888888\n";
  Node2d* node = &dp_lookup_table_[grid_x][grid_y];
  if (node->iterations() != iterations_) {
    node->set_h(CalcHeuCost(grid_x, grid_y));
    node->set_g(kInfiniteCost);
    node->set_pre_node(nullptr);
    node->set_heap_index(0);
    node->set_iterations(iterations_);
  }
  return node;
}

bool Voronoi::IsValidCell(const int grid_x, const int grid_y) const {
  if (!IsWithinMap(grid_x, grid_y)) {
    return false;
  }
  if (gvd_map_[grid_x][grid_y].dist < 0) { // circumscribed_radius_
    return false;
  }
  return true;
}

bool Voronoi::IsValidCellStartAndEnd(int grid_x, int grid_y) const {

  if (!IsWithinMap(grid_x, grid_y)) {
    std::cout << "1111111111111111111111111111111111\n";
    return false;
  }

  if (gvd_map_[grid_x][grid_y].dist < 0) {
    return false;
  }
  return true;
}


int Voronoi::CalcHeuCost(const int grid_x, const int grid_y) const {
  if (end_node_ == nullptr) {
    return 0;
  }
  return 10 * std::max(std::abs(grid_x - end_node_->grid_x()),
                       std::abs(grid_y - end_node_->grid_y()));
}

bool Voronoi::SetStartAndEndConfiguration(int sx, int sy, int ex,
                                                 int ey) {

  if (!SetStart(sx, sy)) {
    std::cout << "VoronoiPlanner is called on invalid start (" << sx << "," << sy << ")" << std::endl;
    return false;
  }
  // CHECK_NOTNULL(start_node_);
  // since the goal has not been set yet, the start node's h value is set to 0
  // CHECK_EQ(start_node_->h(), 0);

  if (!SetEnd(ex, ey)) {
    std::cout << "VoronoiPlanner is called on invalid end (" << ex << "," << ey << ")" << std::endl;
    return false;
  }
  // CHECK_NOTNULL(end_node_);
  // CHECK_EQ(end_node_->h(), 0);
  return true;
}

void Voronoi::ComputeGridSearchActions() {
  // initialize some constants for 2D search
  // 初始化常量来2D搜索
  actions_.dx[0] = 1;
  actions_.dy[0] = 1;
  actions_.dx[1] = 1;
  actions_.dy[1] = 0;
  actions_.dx[2] = 1;
  actions_.dy[2] = -1;
  actions_.dx[3] = 0;
  actions_.dy[3] = 1;
  actions_.dx[4] = 0;
  actions_.dy[4] = -1;
  actions_.dx[5] = -1;
  actions_.dy[5] = 1;
  actions_.dx[6] = -1;
  actions_.dy[6] = 0;
  actions_.dx[7] = -1;
  actions_.dy[7] = -1;

  // compute distances
  // 计算距离
  for (int dind = 0; dind < kNumOfGridSearchActions; dind++) {
    if (actions_.dx[dind] != 0 && actions_.dy[dind] != 0) {
      actions_.dxy_cost[dind] = 14;
    } else {
      actions_.dxy_cost[dind] = 10;
    }
  }
}

bool Voronoi::Search(int sx, int sy, int ex, int ey,
                     std::vector<std::vector<VoronoiData>>&& gvd_map,
                     std::vector<std::pair<int, int>>* path) {
  std::cout << "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n";
  
  if (!initialized_) {// 如果未初始化，则输出错误信息
    // LOG(ERROR) << "VoronoiPlanner has not been initialized.";
    std::cout << "VoronoiPlanner has not been initialized." << std::endl;
    return false;
  }
  std::cout << "ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n";
  
  // Sanity checks.
  // CHECK_NOTNULL(path);
  // 检查合法性
  const auto start_timestamp = std::chrono::system_clock::now();

  gvd_map_ = std::move(gvd_map);

  // Find grid path from start to Voronoi edges.
  // 从起点到Voronoi边缘找到网格路径
  int voronoi_start_x = 0;
  int voronoi_start_y = 0;
  GridSearchResult result1;

  std::cout << "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee\n";
  // 定位到此处的问题
  if (!SearchPathToVoronoiEdges(sx, sy, ex, ey, &voronoi_start_x,
                                &voronoi_start_y, &result1)) {
    std::cout << "Failed to find grid path from start to Voronoi edges." << std::endl;
    return false;
  }
  std::cout << "fffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n";

  if (voronoi_start_x == ex && voronoi_start_y == ey) {
    *path = result1.grid_path;
    return true;
  }

  // Find grid path from end to Voronoi edges.
  // 从终点到Voronoi边缘找到网格路径
  int voronoi_goal_x = 0;
  int voronoi_goal_y = 0;
  GridSearchResult result2;
  if (!SearchPathToVoronoiEdges(ex, ey, sx, sy, &voronoi_goal_x,
                                &voronoi_goal_y, &result2)) {
    // LOG(ERROR) << "Failed to find grid path from end to Voronoi edges.";
    std::cout << "Failed to find grid path from end to Voronoi edges." << std::endl;
    return false;
  }
  std::reverse(result2.grid_path.begin(), result2.grid_path.end());

  if (voronoi_goal_x == sx && voronoi_goal_y == sy) {
    *path = result2.grid_path;
    return true;
  }

  // Find grid path along Voronoi edges.
  // 沿着Voronoi边缘找到网格路径
  GridSearchResult result3;
  if (!SearchPathAlongVoronoiEdges(voronoi_start_x, voronoi_start_y,
                                   voronoi_goal_x, voronoi_goal_y, &result3)) {
    // LOG(ERROR) << "Failed to find grid path along Voronoi edges.";
    std::cout << "Failed to find grid path along Voronoi edges." << std::endl;
    return false;
  }

  path->clear();
  path->insert(path->end(), std::make_move_iterator(result1.grid_path.begin()),
               std::make_move_iterator(result1.grid_path.end()));
  path->insert(path->end(), std::make_move_iterator(result3.grid_path.begin()),
               std::make_move_iterator(result3.grid_path.end()));
  path->insert(path->end(), std::make_move_iterator(result2.grid_path.begin()),
               std::make_move_iterator(result2.grid_path.end()));

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  //LOG(INFO) << std::fixed << "Total time used to search Voronoi grid path: "
  //          << diff.count() * 1e3 << " ms.";

  return true;
}

bool Voronoi::SearchPathToVoronoiEdges(int sx, int sy, int ex, int ey,
                                       int* voronoi_goal_x,
                                       int* voronoi_goal_y,
                                       GridSearchResult* result) {
  std::cout << "ppppppppppppppppppppppppppppppppppppp\n";
  
  const auto start_timestamp = std::chrono::system_clock::now();
  // clean up previous planning result
  Clear();
  std::cout << "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz\n";
  
  search_type_ = SearchType::DP;

  need_check_voronoi_ = false; 

  ++iterations_;
  std::cout << "###########################################\n";
  if (!SetStartAndEndConfiguration(sx, sy, ex, ey)) {
    std::cout << "###########################################\n";
    return false;
  }
  std::cout << "+++++++++++++++++++++++++++++++++++++++++++++\n";

  // initialize start node and insert it into heap
  start_node_->set_g(0);
  start_node_->set_h(CalcHeuCost(sx, sy));
  std::cout << CalcHeuCost(sx, sy) << std::endl;
  open_list_->Insert(start_node_, GetKey(start_node_));
  std::cout << "----------------------------------------------\n";
  // grid search begins
  std::size_t explored_node_num = 0U;
  std::cout << "hihihihihihih\n";
  std::cout << open_list_->Empty() << std::endl;
  std::cout << open_list_->Empty() << std::endl;
  std::cout << "hihihihihihih\n";

  while (!open_list_->Empty()) {
    std::cout << "11111111111111111111111111\n";
    auto* node = dynamic_cast<Node2d*>(open_list_->Pop());
    std::cout << "22222222222222222222222222\n";
    //CHECK_NOTNULL(node);
    //CHECK_NE(node->g(), common::kInfiniteCost);
    closed_list_[CalcGridXYIndex(node->grid_x(), node->grid_y())] =
        Node::NodeStatus::CLOSED;
    std::cout << "33333333333333333333333333\n";
    std::cout << "33333333333333333333333333\n";
    std::cout << "33333333333333333333333333\n";
    std::cout << "33333333333333333333333333\n";
    int curr_x = node->grid_x();
    int curr_y = node->grid_y();

    if ((curr_x == ex && curr_y == ey) ||
        (gvd_map_[curr_x][curr_y].is_voronoi)) {
      *voronoi_goal_x = curr_x;
      *voronoi_goal_y = curr_y;
      break;
    }

    // new expand
    ++explored_node_num;
    UpdateSuccs(node);
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  //LOG(INFO) << std::fixed << "Time used to search path to Voronoi edges: "
  //          << diff.count() * 1e3 << " ms.";

  LoadGridSearchResult(*voronoi_goal_x, *voronoi_goal_y, result);
  return true;
}

bool Voronoi::SearchPathAlongVoronoiEdges(int sx, int sy, int ex, int ey,
                                          GridSearchResult* result) {
  const auto start_timestamp = std::chrono::system_clock::now();

  // clean up previous planning result
  Clear();

  search_type_ = SearchType::A_STAR;
  //search_type_ = SearchType::DP;
  need_check_voronoi_ = true;

  ++iterations_;

  if (!SetStartAndEndConfiguration(sx, sy, ex, ey)) {
    return false;
  }

  // initialize start node and insert it into heap
  start_node_->set_g(0);
  start_node_->set_h(CalcHeuCost(sx, sy));
  open_list_->Insert(start_node_, GetKey(start_node_));

  // grid search begins
  std::size_t explored_node_num = 0U;
  while (!open_list_->Empty() && end_node_->g() > open_list_->GetMinKey()) {
    auto* node = dynamic_cast<Node2d*>(open_list_->Pop());
    //CHECK_NOTNULL(node);
    //CHECK_NE(node->g(), common::kInfiniteCost);
    closed_list_[CalcGridXYIndex(node->grid_x(), node->grid_y())] =
        Node::NodeStatus::CLOSED;

    // new expand
    ++explored_node_num;
    UpdateSuccs(node);
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  //LOG(INFO) << std::fixed << "Time used to search path along Voronoi edges: "
   //         << diff.count() * 1e3 << " ms.";

  if (end_node_->g() == kInfiniteCost) {
    // LOG(ERROR) << "VoronoiPlanner returns infinite cost (open_list run out)";
    std::cout << "VoronoiPlanner returns infinite cost (open_list run out)" << std::endl;
    return false;
  }

  LoadGridSearchResult(end_node_->grid_x(), end_node_->grid_y(), result);
  return true;
}

void Voronoi::UpdateSuccs(const Node2d* curr_node) {
  //CHECK_NOTNULL(curr_node);
  const int curr_x = curr_node->grid_x();
  const int curr_y = curr_node->grid_y();
  //CHECK(CheckVoronoi(curr_x, curr_y));
  //CHECK(IsValidCell(curr_x, curr_y));

  for (int action_id = 0; action_id < kNumOfGridSearchActions;
       ++action_id) {
    const int succ_x = curr_x + actions_.dx[action_id];
    const int succ_y = curr_y + actions_.dy[action_id];
    if (!IsValidCell(succ_x, succ_y)) {
      continue;
    }
    if (closed_list_[CalcGridXYIndex(succ_x, succ_y)] ==
        Node::NodeStatus::CLOSED) {
      continue;
    }
    if (!CheckVoronoi(succ_x, succ_y)) {
      continue;
    }
    // get action cost
    int action_cost = GetActionCost(curr_x, curr_y, action_id);

    Node2d* succ_node = GetNode(succ_x, succ_y);
    // see if we can decrease the value of successive node taking into account
    // the cost of action
    if (succ_node->g() > curr_node->g() + action_cost) {
      succ_node->set_g(curr_node->g() + action_cost);
      succ_node->set_pre_node(curr_node);

      // re-insert into heap if not closed yet
      if (succ_node->heap_index() == 0) {
        open_list_->Insert(succ_node, GetKey(succ_node));
      } else {
        open_list_->Update(succ_node, GetKey(succ_node));
      }
    }
  }
}

int Voronoi::GetActionCost(int curr_x, int curr_y, int action_id) const {
  //CHECK(IsValidCell(curr_x, curr_y));
  const int succ_x = curr_x + actions_.dx[action_id];
  const int succ_y = curr_y + actions_.dy[action_id];
  // CHECK(IsValidCell(succ_x, succ_y));
  //if (!IsValidCell(succ_x, succ_y)) {
  //  std::cout << "error" << std::endl;
  //}
  return actions_.dxy_cost[action_id];
}

void Voronoi::LoadGridSearchResult(int end_x, int end_y,
                                   GridSearchResult* result) const {
  if (result == nullptr) {
    return;
  }

  result->path_cost = dp_lookup_table_[end_x][end_y].g();
  const Node2d* node = &dp_lookup_table_[end_x][end_y];
  std::vector<std::pair<int, int>> grid_path;
  while (node != nullptr) {
    grid_path.emplace_back(node->grid_x(), node->grid_y());
    node = node->pre_node();
  }
  std::reverse(grid_path.begin(), grid_path.end());
  result->grid_path = std::move(grid_path);
}

}