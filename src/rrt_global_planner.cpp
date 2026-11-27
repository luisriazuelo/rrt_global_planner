#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>
//
#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "rrt_global_planner/rrt_global_planner.hpp"

namespace rrt_global_planner
{

double distance(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1){
    return std::sqrt((int)(x1-x0)*(int)(x1-x0) + (int)(y1-y0)*(int)(y1-y0));
}



void RRTPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock(); 
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_samples", rclcpp::ParameterValue(
      30000.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".dist_th", rclcpp::ParameterValue(
      10.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_dist", rclcpp::ParameterValue(
      0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".seed", rclcpp::ParameterValue(-1));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_cost", rclcpp::ParameterValue(253));

  node_->get_parameter(name_ + ".max_samples", max_samples_);
  node_->get_parameter(name_ + ".dist_th", dist_th_);
  node_->get_parameter(name_ + ".max_dist", max_dist_);
  node_->get_parameter(name_ + ".seed", seed_);

  node_->get_parameter(name_ + ".max_cost", max_cost_); 

  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "rrt_markers", rclcpp::SystemDefaultsQoS());

}

void RRTPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type RRTPlanner",
    name_.c_str());
}

void RRTPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type RRTPlanner",
    name_.c_str());
    marker_pub_->on_activate();

}

void RRTPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type RRTPlanner",
    name_.c_str());
    marker_pub_->on_deactivate();

}

nav_msgs::msg::Path RRTPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  std::cout << "RRTPlanner::createPlan" << std::endl;
    

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

  // Get start and goal poses in map coordinates
  unsigned int goal_mx, goal_my, start_mx, start_my;
   if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)){
    RCLCPP_INFO(
      node_->get_logger(), "Goal position is out of map bounds.");
    return global_path;
    }    
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);

    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    std::vector<int> point_start{(int)start_mx,(int)start_my};
    std::vector<int> point_goal{(int)goal_mx,(int)goal_my};    
  	std::vector<std::vector<int>> solRRT;
    bool computed = computeRRT(point_start, point_goal, solRRT);
    //computed = true;
    if (computed){   
      getPlan(solRRT, global_path);
      // Add goal to plan
      global_path.poses.push_back(goal);
    }else{
        RCLCPP_INFO(
      node_->get_logger(), "No plan computed");
    }

  return global_path;
}

// ---------------------------------------------------------------------------
// RRTPlanner::computeRRT
// ---------------------------------------------------------------------------

bool RRTPlanner::computeRRT(const std::vector<int> start, const std::vector<int> goal,
                            std::vector<std::vector<int>>& sol)
{
    sol.clear();

    // Initialize random number generator
    std::mt19937 gen(seed_ >= 0 ? static_cast<unsigned int>(seed_) : std::random_device{}());
    const int size_x = static_cast<int>(costmap_->getSizeInCellsX());
    const int size_y = static_cast<int>(costmap_->getSizeInCellsY());
    std::uniform_int_distribution<int> rand_x(0, size_x - 1);
    std::uniform_int_distribution<int> rand_y(0, size_y - 1);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    // Parameters are in meters: convert them to cells
    const double res = costmap_->getResolution();
    const double max_step = std::max(1.0, max_dist_ / res);   // Steer step (at least one cell)
    const double dist_th_cells = dist_th_ / res;              // goal distance threshold

    // Initialize the tree with the starting point in map coordinates
    std::vector<TreeNode> tree;
    tree.push_back({start[0], start[1], -1});

    bool finished = false;
    int goal_node = -1;   // index of the tree node that connects to the goal

    // Trivial case: the goal is already reachable from the start
    if (distance(start[0], start[1], goal[0], goal[1]) < dist_th_cells &&
        obstacleFree(start[0], start[1], goal[0], goal[1]))
    {
        finished = true;
        goal_node = 0;
    }

    const int max_it = static_cast<int>(max_samples_);
    int it = 0;
    for (it = 0; it < max_it && !finished; ++it) {

        // 1. SampleFree: goal with a random free cell
  
        // 2. Nearest node of the tree (squared distance, no sqrt needed)
 
        // 3. Steer: move from near towards the sample at most max_step cells

        // 4. ObstacleFree: add the node only if the segment near-new is collision free
 
        // 5. Is the goal reachable from the new node?
       
    }

    // Build the solution: goal side -> start side, without the start node (root)
    if (finished) {
        for (int i = goal_node; tree[i].parent != -1; i = tree[i].parent) {
            sol.push_back({tree[i].x, tree[i].y});
        }
    }

    // Metrics for the log of createPlan
    last_iters_ = it;
    last_nodes_ = static_cast<int>(tree.size());

    // Visualization in RViz
    visualizeTree(tree);

    return finished;
}


bool RRTPlanner::obstacleFree(const unsigned int x0, const unsigned int y0, 
                            const unsigned int x1, const unsigned int y1){
    //Bresenham algorithm to check if the line between points (x0,y0) - (x1,y1) is free of collision

    int dx = x1 - x0;
    int dy = y1 - y0;

    int incr_x = (dx > 0) ? 1.0 : -1.0;
    int incr_y = (dy > 0) ? 1.0 : -1.0;

    unsigned int da, db, incr_x_2, incr_y_2;
    if (abs(dx) >= abs(dy)){
        da = abs(dx); db = abs(dy);
        incr_x_2 = incr_x; incr_y_2 = 0;
    }else{
        da = abs(dy); db = abs(dx);
        incr_x_2 = 0; incr_y_2 = incr_y;
    }

    int p = 2*db - da;
    unsigned int a = x0; 
    unsigned int b = y0;
    unsigned int end = da;
    for (unsigned int i=0; i<end; i++){
        if (costmap_->getCost(a, b) >= max_cost_){  
            return false;
        }else{
            if (p >= 0){
                a += incr_x;
                b += incr_y;
                p -= 2*da;
            }else{
                a += incr_x_2;
                b += incr_y_2;
            }
            p += 2*db;
        }
    }

    return true;
}

void RRTPlanner::getPlan(const std::vector<std::vector<int>> sol, nav_msgs::msg::Path& plan){

    for (auto it = sol.rbegin(); it != sol.rend(); it++){
      std::vector<int> point = (*it);
      geometry_msgs::msg::PoseStamped pose;
 
      costmap_->mapToWorld((unsigned int)point[0], (unsigned int)point[1], 
                            pose.pose.position.x, pose.pose.position.y);
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      plan.poses.push_back(pose);                            
    }
}

void RRTPlanner::visualizeTree(const std::vector<TreeNode> & tree)
{
  visualization_msgs::msg::Marker m;
  fillMarkerMsg(global_frame_, "full_tree", 4,
    visualization_msgs::msg::Marker::LINE_LIST, m);
  m.scale.x = 0.02;
  m.color.r = 0.0; m.color.g = 0.6; m.color.b = 0.0; m.color.a = 0.4;

  for (size_t i = 1; i < tree.size(); ++i) {
    if (tree[i].parent < 0) {
      continue;
    }
    geometry_msgs::msg::Point a, b;
    costmap_->mapToWorld(
      static_cast<unsigned int>(tree[tree[i].parent].x),
      static_cast<unsigned int>(tree[tree[i].parent].y), a.x, a.y);
    costmap_->mapToWorld(
      static_cast<unsigned int>(tree[i].x),
      static_cast<unsigned int>(tree[i].y), b.x, b.y);
    m.points.push_back(a);
    m.points.push_back(b);
  }
  marker_pub_->publish(m);   // same id on every call -> RViz replaces the old tree
}

void RRTPlanner::fillMarkerMsg(const std::string frame_id, const std::string ns, const int id,
                    const int type, visualization_msgs::msg::Marker& msg){
    
    msg.header.frame_id = frame_id;
    msg.header.stamp = node_->now();
    msg.ns = ns;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.id = id;
    msg.type = type;
}

}  // namespace rrt_global_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rrt_global_planner::RRTPlanner, nav2_core::GlobalPlanner)
