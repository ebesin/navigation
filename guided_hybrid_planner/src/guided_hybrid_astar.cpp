/*
 * @Author       : dwayne
 * @Date         : 2023-04-08
 * @LastEditTime : 2023-04-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "guided_hybrid_astar.hpp"

namespace guided_hybrid_a_star {
GuidedHybridAStar::GuidedHybridAStar(nav2_costmap_2d::Costmap2D* costmap, const SearchInfo& serch_info)
{
    this->costmap_     = costmap;
    this->x_size_      = this->costmap_->getSizeInCellsX();
    this->y_size_      = this->costmap_->getSizeInCellsY();
    this->unit_angle_  = 2 * M_PI / serch_info.angle_segment_size;
    this->serch_info_  = serch_info;
    this->rs_path_ptr_ = std::make_shared<RSPath>(serch_info.wheel_base / tan(serch_info.max_steer_angle));
    this->downsampler_ = std::make_unique<CostmapDownsampler>();
    initMotion();
    // graph_.reserve(x_size_ * y_size_ * angle_size_);
}

GuidedHybridAStar::GuidedHybridAStar(int width, int height, int dim3, const SearchInfo& serch_info)
{
    this->x_size_      = width;
    this->y_size_      = height;
    this->angle_size_  = dim3;
    this->unit_angle_  = 2 * M_PI / dim3;
    this->serch_info_  = serch_info;
    this->rs_path_ptr_ = std::make_shared<RSPath>(serch_info.wheel_base / tan(serch_info.max_steer_angle));
    // graph_.reserve(x_size_ * y_size_ * angle_size_);
}

void GuidedHybridAStar::resetAll()
{
    this->x_size_     = this->costmap_->getSizeInCellsX();
    this->y_size_     = this->costmap_->getSizeInCellsY();
    this->unit_angle_ = 2 * M_PI / serch_info_.angle_segment_size;
    std::priority_queue<std::pair<StateNodePtr, float>, std::vector<std::pair<StateNodePtr, float>>, cmp> temp_list;
    std::swap(temp_list, open_list_);
    graph_.clear();
    close_list_.clear();
    resetObstacleHeuristic(costmap_, start_, goal_);
    initMotion();
}

unsigned int GuidedHybridAStar::getAngleIndex(const double heading) const
{
    int angle_index = static_cast<int>(heading / unit_angle_);
    if (angle_index < 0) angle_index += serch_info_.angle_segment_size;
    if (angle_index >= serch_info_.angle_segment_size) angle_index -= serch_info_.angle_segment_size;
    return angle_index;
}


unsigned int GuidedHybridAStar::state2Index(const Vec3d& state) const
{
    // Vec3i index;
    unsigned int x     = static_cast<int>(state[0]);
    unsigned int y     = static_cast<int>(state[1]);
    unsigned int theta = static_cast<int>(getAngleIndex(state[2]));
    return x * serch_info_.angle_segment_size + y * x_size_ * serch_info_.angle_segment_size + theta;
}

void GuidedHybridAStar::setSerchInfo(const SearchInfo& serch_info)
{
    this->serch_info_  = serch_info;
    this->unit_angle_  = 2 * M_PI / serch_info.angle_segment_size;
    this->rs_path_ptr_ = std::make_shared<RSPath>(serch_info.wheel_base / tan(serch_info.max_steer_angle));
    initMotion();
}

void GuidedHybridAStar::setStart(const Vec3d& start_state)
{
    start_ = std::make_shared<StateNode>();
    start_->setIndex(state2Index(start_state));
    start_->setState(start_state);
}

void GuidedHybridAStar::setGoal(const Vec3d& start_state)
{
    goal_ = std::make_shared<StateNode>();
    goal_->setIndex(state2Index(start_state));
    goal_->setState(start_state);
}

bool GuidedHybridAStar::beyondBoundary(const Vec2d& pos) const
{
    return pos.x() < 0 || pos.x() > x_size_ || pos.y() < 0 || pos.y() > y_size_;
}

void GuidedHybridAStar::initMotion()
{
    forward_projections_.clear();
    backward_projections_.clear();
    int steer_angle_segment_size = serch_info_.steer_angle_segment_size;
    if (steer_angle_segment_size % 2 == 0) steer_angle_segment_size += 1;
    // if (steer_angle_segment_size / 2 % 2 != 0) steer_angle_segment_size += 2;

    double segment_angle = static_cast<double>(2 * serch_info_.max_steer_angle / (steer_angle_segment_size - 1));
    forward_projections_.resize(steer_angle_segment_size);
    backward_projections_.resize(steer_angle_segment_size);
    for (int i = 0; i < steer_angle_segment_size / 2; i++) {
        double R       = static_cast<float>(serch_info_.wheel_base / (tan(serch_info_.max_steer_angle - segment_angle * i)));
        double angle   = 2 * asin(sqrt(2.0) / (2 * R));
        double delta_x = R * sin(angle);
        double delta_y = R - R * cos(angle);
        std::cout << "angle:" << angle << " delta_x:" << delta_x << " delta_y:" << delta_y << std::endl;
        forward_projections_[i]                                 = MotionPose(delta_x, delta_y, angle);
        forward_projections_[steer_angle_segment_size - 1 - i]  = MotionPose(delta_x, -delta_y, -angle);
        backward_projections_[i]                                = MotionPose(-delta_x, delta_y, -angle);
        backward_projections_[steer_angle_segment_size - 1 - i] = MotionPose(-delta_x, -delta_y, angle);
    }
    forward_projections_[static_cast<int>(steer_angle_segment_size / 2)]  = MotionPose(sqrt(2), 0, 0);
    backward_projections_[static_cast<int>(steer_angle_segment_size / 2)] = MotionPose(-sqrt(2), 0, 0);
}

bool GuidedHybridAStar::getFrontNeighbor(const StateNodePtr& current_node, unsigned int neighbour_index, StateNodePtr& next_node)
{
    Vec3d  current_state   = current_node->getState();
    double current_heading = current_state[2];

    MotionPose pose        = forward_projections_[neighbour_index];
    double     new_heading = current_heading + pose._theta;
    if (new_heading < 0.0) new_heading += 2 * M_PI;
    if (new_heading > 2 * M_PI) new_heading -= 2 * M_PI;
    Vec3d next_state;
    //旋转矩阵
    Eigen::Matrix2d rotation;
    rotation << cos(current_heading), -sin(current_heading), sin(current_heading), cos(current_heading);
    //根据当前航向角计算出的delta_x和delta_y
    Vec2d delta = rotation * Vec2d(pose._x, pose._y);

    double neighbour_x = current_state[0] + delta[0];
    double neighbour_y = current_state[1] + delta[1];
    // if (!beyondBoundary(Vec2d(neighbour_x, neighbour_y)) &&
    //     (!collision_checker_->inCollision(neighbour_x, neighbour_y, getAngleIndex(new_heading), serch_info_.travel_unknown))) {
    if ((!collision_checker_->inCollision(neighbour_x, neighbour_y, getAngleIndex(new_heading), serch_info_.travel_unknown))) {
        Vec3d neighbour_state = Vec3d(neighbour_x, neighbour_y, new_heading);
        next_node             = std::make_shared<StateNode>(state2Index(neighbour_state));
        next_node->setState(neighbour_state);
        return true;
    }
    return false;
}

bool GuidedHybridAStar::getBackNeighbor(const StateNodePtr& current_node, unsigned int neighbour_index, StateNodePtr& next_node)
{
    Vec3d  current_state   = current_node->getState();
    double current_heading = current_state[2];


    MotionPose pose        = backward_projections_[neighbour_index];
    double     new_heading = current_heading + pose._theta;
    if (new_heading < 0.0) new_heading += 2 * M_PI;
    if (new_heading > 2 * M_PI) new_heading -= 2 * M_PI;
    Vec3d next_state;
    //旋转矩阵
    Eigen::Matrix2d rotation;
    rotation << cos(current_heading), -sin(current_heading), sin(current_heading), cos(current_heading);
    //根据当前航向角计算出的delta_x和delta_y
    Vec2d  delta       = rotation * Vec2d(pose._x, pose._y);
    double neighbour_x = current_state[0] + delta[0];
    double neighbour_y = current_state[1] + delta[1];
    if (!beyondBoundary(Vec2d(neighbour_x, neighbour_y)) &&
        (!collision_checker_->inCollision(neighbour_x, neighbour_y, getAngleIndex(new_heading), serch_info_.travel_unknown))) {
        Vec3d neighbour_state = Vec3d(neighbour_x, neighbour_y, new_heading);
        next_node             = std::make_shared<StateNode>(state2Index(neighbour_state));
        next_node->setState(neighbour_state);
        return true;
    }
    return false;
}


inline float distanceHeuristic2D(const unsigned int idx,
                                 const unsigned int size_x,
                                 const unsigned int target_x,
                                 const unsigned int target_y)
{
    int dx = static_cast<int>(idx % size_x) - static_cast<int>(target_x);
    int dy = static_cast<int>(idx / size_x) - static_cast<int>(target_y);
    return std::sqrt(dx * dx + dy * dy);
}

double GuidedHybridAStar::computeTrajCost(const StateNodePtr& current_node_ptr, const StateNodePtr& neighbor_node_ptr)
{
    unsigned int current_angle_index  = getAngleIndex(current_node_ptr->getState().z());
    unsigned int neighbor_angle_index = getAngleIndex(neighbor_node_ptr->getState().z());
    double       segment_length       = sqrt(2);
    float steering_change_penalty     = (current_angle_index == neighbor_angle_index) ? 1 : serch_info_.steering_change_penalty;
    float steering_penalty            = ((neighbor_angle_index - current_angle_index) == 0) ? 1 : serch_info_.steering_penalty;
    float reverse_penalty             = (neighbor_node_ptr->getDirection() == FORWARD) ? 1 : serch_info_.reverse_penalty;

    return segment_length * reverse_penalty * steering_penalty * steering_change_penalty;
}

double GuidedHybridAStar::computeDistanceHeuristicCost(const StateNodePtr& current_node, const StateNodePtr& target_node)
{
    double h;
    // L2
    h = (current_node->getState().head(2) - target_node->getState().head(2)).norm();

    // L1
    // h = (current_node->getState().head(2) - target_node->getState().head(2)).lpNorm<1>();
    if (h < serch_info_.shot_distance * 5) {
        h = rs_path_ptr_->Distance(current_node->getState().x(),
                                   current_node->getState().y(),
                                   current_node->getState().z(),
                                   target_node->getState().x(),
                                   target_node->getState().y(),
                                   target_node->getState().z());
    }
    return h;
}

void GuidedHybridAStar::resetObstacleHeuristic(nav2_costmap_2d::Costmap2D* costmap,
                                               const StateNodePtr&         start_node,
                                               const StateNodePtr&         goal_node)
{
    Color::Modifier                         blue(Color::FG_BLUE);
    Flag::Modifier                          flag("guided_hybrid_a_star");
    std::weak_ptr<nav2_util::LifecycleNode> ptr;
    downsampler_->on_configure(ptr, "fake_frame", "fake_topic", costmap, 2.0, true);
    downsampler_->on_activate();
    sampled_costmap_  = downsampler_->downsample(2.0);
    unsigned int size = sampled_costmap_->getSizeInCellsX() * sampled_costmap_->getSizeInCellsY();
    if (obstacle_heuristic_lookup_table_.size() == size) {
        std::fill(obstacle_heuristic_lookup_table_.begin(), obstacle_heuristic_lookup_table_.end(), 0.0);
    }
    else {
        unsigned int obstacle_size = obstacle_heuristic_lookup_table_.size();
        obstacle_heuristic_lookup_table_.resize(size, 0.0);
        std::fill_n(obstacle_heuristic_lookup_table_.begin(), obstacle_size, 0.0);
    }
    obstacle_heuristic_queue_.clear();
    obstacle_heuristic_queue_.reserve(sampled_costmap_->getSizeInCellsX() * sampled_costmap_->getSizeInCellsY());
    std::cout << blue << flag << "getSizeInCellsX:" << sampled_costmap_->getSizeInCellsX()
              << "  getSizeInCellsY:" << sampled_costmap_->getSizeInCellsY() << std::endl;
    std::cout << blue << flag << "4" << std::endl;
    const unsigned int size_x = sampled_costmap_->getSizeInCellsX();
    unsigned int start_x = start_node->getState().x(), start_y = start_node->getState().y(), goal_x = goal_node->getState().x(),
                 goal_y = goal_node->getState().y();
    std::cout << blue << flag << "start_node->getState().x():" << start_node->getState().x()
              << " start_node->getState().y():" << start_node->getState().y()
              << "  goal_node->getState().x():" << goal_node->getState().x()
              << " goal_node->getState().y():" << goal_node->getState().y() << std::endl;
    std::cout << blue << flag << "start_x:" << start_x << " start_y:" << start_y << "  goal_x:" << goal_x << " goal_y:" << goal_y
              << std::endl;
    const unsigned int goal_index = floor(goal_y / 2.0) * size_x + floor(goal_x / 2.0);
    std::cout << blue << flag << "goal_index:" << goal_index << std::endl;
    obstacle_heuristic_queue_.emplace_back(distanceHeuristic2D(goal_index, size_x, start_x, start_y), goal_index);
    obstacle_heuristic_lookup_table_[goal_index] = -0.00001f;
}


double GuidedHybridAStar::computeObstacleHeuristicCost(const StateNodePtr& current_node, const StateNodePtr& target_node)
{
    const unsigned int size_x = sampled_costmap_->getSizeInCellsX();

    double start_x = current_node->getState().x(), start_y = current_node->getState().y(), goal_x = target_node->getState().x(),
           goal_y                          = target_node->getState().y();
    int                start_x_new         = floor(start_x / 2.0);
    int                start_y_new         = floor(start_y / 2.0);
    const unsigned int start_index         = start_y_new * size_x + start_x_new;
    const float&       requested_node_cost = obstacle_heuristic_lookup_table_[start_index];
    if (requested_node_cost > 0.0f) {
        return 2.0 * requested_node_cost;
    }

    for (auto& n : obstacle_heuristic_queue_) {
        n.first = -obstacle_heuristic_lookup_table_[n.second] + distanceHeuristic2D(n.second, size_x, start_x_new, start_y_new);
    }

    // 小顶堆
    std::make_heap(obstacle_heuristic_queue_.begin(), obstacle_heuristic_queue_.end(), ObstacleHeuristicComparator{});

    const int          size_x_int = static_cast<int>(size_x);
    const unsigned int size_y     = sampled_costmap_->getSizeInCellsY();
    const float        sqrt_2     = sqrt(2);
    float              c_cost, cost, travel_cost, new_cost, existing_cost;
    unsigned int       idx, mx, my, mx_idx, my_idx;
    unsigned int       new_idx = 0;


    const std::vector<int> neighborhood = {1,
                                           -1,   // left right
                                           size_x_int,
                                           -size_x_int,   // up down
                                           size_x_int + 1,
                                           size_x_int - 1,   // upper diagonals
                                           -size_x_int + 1,
                                           -size_x_int - 1};   // lower diagonals

    while (!obstacle_heuristic_queue_.empty()) {
        // 找到队列中代价值最小的节点index并弹出队列
        idx = obstacle_heuristic_queue_.front().second;
        std::pop_heap(obstacle_heuristic_queue_.begin(), obstacle_heuristic_queue_.end(), ObstacleHeuristicComparator{});
        obstacle_heuristic_queue_.pop_back();
        c_cost = obstacle_heuristic_lookup_table_[idx];
        if (c_cost > 0.0f) {
            continue;
        }
        c_cost                                = -c_cost;
        obstacle_heuristic_lookup_table_[idx] = c_cost;

        my_idx = idx / size_x;
        mx_idx = idx - (my_idx * size_x);

        for (unsigned int i = 0; i != neighborhood.size(); i++) {
            new_idx = static_cast<unsigned int>(static_cast<int>(idx) + neighborhood[i]);

            if (new_idx < size_x * size_y) {
                cost = static_cast<float>(sampled_costmap_->getCost(new_idx));
                if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                    continue;
                }

                my = new_idx / size_x;
                mx = new_idx - (my * size_x);

                if (mx == 0 && mx_idx >= size_x - 1 || mx >= size_x - 1 && mx_idx == 0) {
                    continue;
                }
                if (my == 0 && my_idx >= size_y - 1 || my >= size_y - 1 && my_idx == 0) {
                    continue;
                }

                existing_cost = obstacle_heuristic_lookup_table_[new_idx];
                if (existing_cost <= 0.0f) {
                    travel_cost = ((i <= 3) ? 1.0f : sqrt_2) * (1.0f + (serch_info_.cost_penalty * cost / 252.0f));
                    new_cost    = c_cost + travel_cost;
                    if (existing_cost == 0.0f || -existing_cost > new_cost) {
                        obstacle_heuristic_lookup_table_[new_idx] = -new_cost;
                        obstacle_heuristic_queue_.emplace_back(new_cost + distanceHeuristic2D(new_idx, size_x, start_x, start_y),
                                                               new_idx);
                        std::push_heap(
                            obstacle_heuristic_queue_.begin(), obstacle_heuristic_queue_.end(), ObstacleHeuristicComparator{});
                    }
                }
            }
        }

        if (idx == start_index) {
            RCLCPP_INFO(logger_, "==============================================");
            break;
        }
    }
    return abs(2.0 * requested_node_cost);
}

double GuidedHybridAStar::computeHeuristicCost(const StateNodePtr& current_node, const StateNodePtr& target_node)
{
    return std::max(computeDistanceHeuristicCost(current_node, target_node),
                    computeObstacleHeuristicCost(current_node, target_node));
    // return computeDistanceHeuristicCost(current_node, target_node);
}



bool GuidedHybridAStar::analyticExpansions(const StateNodePtr& current_node, const StateNodePtr& target_node, double& distance)
{
    VectorVec3d rs_path_poses =
        rs_path_ptr_->GetRSPath(current_node->getState(), target_node->getState(), serch_info_.move_step_distance, distance);
    for (const auto& pose : rs_path_poses)
        if (beyondBoundary(pose.head(2)) ||
            collision_checker_->inCollision(pose.x(), pose.y(), getAngleIndex(pose.z()), serch_info_.travel_unknown)) {
            return false;
        };
    rs_path_poses.erase(rs_path_poses.begin());
    target_node->setIntermediateStates(rs_path_poses);
    target_node->setParentNode(current_node);
}


bool GuidedHybridAStar::complutePath()
{
    resetAll();
    unsigned int iterations = 0;
    start_->setParentNode(nullptr);
    start_->setTrajCost(0.0);
    start_->setHeuCost(computeHeuristicCost(start_, goal_));
    open_list_.emplace(start_, start_->getCost());
    graph_.emplace(start_->getIndex(), start_);
    start_->setStatus(IN_OPENSET);
    StateNodePtr current_node_ptr;
    StateNodePtr neighbor_node_ptr;
    rclcpp::Rate r(100);
    while (!open_list_.empty() && iterations < serch_info_.max_iterations) {
        // r.sleep();
        // RCLCPP_INFO(logger_, "open_list_size:%d", open_list_.size());
        /* code */
        current_node_ptr = open_list_.top().first;
        open_list_.pop();
        //如果该节点在closelist中就不访问(发生在该节点的代价已经被更小的值更新时)
        if (current_node_ptr->getStatus() == IN_CLOSESET) {
            std::cout << "current_node_ptr->getStatus(): "
                      << "IN_CLOSESET" << std::endl;
            continue;
        }
        iterations++;
        // RCLCPP_INFO(logger_, "iterations: %d", iterations);
        // std::cout << "iterations: " << iterations << std::endl;
        current_node_ptr->setStatus(IN_CLOSESET);
        close_list_.emplace(current_node_ptr->getIndex(), current_node_ptr);
        if ((current_node_ptr->getState().head(2) - goal_->getState().head(2)).norm() <= serch_info_.shot_distance) {
            std::cout << "analyticExpansions" << std::endl;
            double rs_length = 0.0;
            if (analyticExpansions(current_node_ptr, goal_, rs_length)) {
                goal_->setParentNode(current_node_ptr);
                return true;
            }
        }
        // RCLCPP_INFO(logger_,
        //             "x:%-12.5fy:%-12.5fheading:%-12.5fheu_cost:%-12.5ftraj_cost:%-12.5ftotal_cost:%-12.5f",
        //             current_node_ptr->getState().x(),
        //             current_node_ptr->getState().y(),
        //             current_node_ptr->getState().z() * 180 / M_PI,
        //             current_node_ptr->getHeuCost(),
        //             current_node_ptr->getTrajcost(),
        //             current_node_ptr->getCost());

        // for (int i = 0; i < serch_info_.steer_angle_segment_size; i++) {
        for (int i = 0; i < forward_projections_.size(); i++) {
            //获取所有合法的前方邻居节点
            if (getFrontNeighbor(current_node_ptr, i, neighbor_node_ptr)) {
                std::unordered_map<int, GuidedHybridAStar::StateNodePtr>::iterator iterator =
                    graph_.find(neighbor_node_ptr->getIndex());
                if (iterator == graph_.end()) {
                    //说明是未访问过的节点
                    neighbor_node_ptr->setStatus(IN_OPENSET);
                    neighbor_node_ptr->setDirection(FORWARD);
                    double traj_cost = current_node_ptr->getTrajcost() + computeTrajCost(current_node_ptr, neighbor_node_ptr);
                    neighbor_node_ptr->setTrajCost(traj_cost);
                    // neighbor_node_ptr->setHeuCost(computeHeuristicCost(neighbor_node_ptr, goal_));
                    double dist_cost = computeDistanceHeuristicCost(neighbor_node_ptr, goal_);
                    double obs_cost  = computeObstacleHeuristicCost(neighbor_node_ptr, goal_);
                    neighbor_node_ptr->setHeuCost(std::max(dist_cost, obs_cost));
                    // neighbor_node_ptr->setHeuCost(dist_cost);
                    neighbor_node_ptr->setParentNode(current_node_ptr);

                    // RCLCPP_INFO(logger_,
                    //             "x:%-12.5fy:%-12.5fheading:%-12.5fheu_cost:%-12.5ftraj_cost:%-12.5ftotal_cost:%-12.5fdist_"
                    //             "cost:%-12.5fobs_cost:%-12.5f",
                    //             neighbor_node_ptr->getState().x(),
                    //             neighbor_node_ptr->getState().y(),
                    //             neighbor_node_ptr->getState().z() * 180 / M_PI,
                    //             neighbor_node_ptr->getHeuCost(),
                    //             neighbor_node_ptr->getTrajcost(),
                    //             neighbor_node_ptr->getCost(),
                    //             dist_cost,
                    //             obs_cost);
                    if (visualization_tools_ptr_) {
                        visualization_tools_ptr_->add_point_pair(
                            Vec2d(costmap_->getOriginX() + (neighbor_node_ptr->getState().x() + 0.5) * costmap_->getResolution(),
                                  costmap_->getOriginY() + (neighbor_node_ptr->getState().y() + 0.5) * costmap_->getResolution()),
                            Vec2d(costmap_->getOriginX() + (current_node_ptr->getState().x() + 0.5) * costmap_->getResolution(),
                                  costmap_->getOriginY() + (current_node_ptr->getState().y() + 0.5) * costmap_->getResolution()));
                        visualization_tools_ptr_->publish_point_pairs();
                    }
                    graph_.emplace(neighbor_node_ptr->getIndex(), neighbor_node_ptr);
                    open_list_.emplace(neighbor_node_ptr, neighbor_node_ptr->getCost());
                }
                else if (iterator->second->getStatus() == IN_OPENSET) {
                    //说明已经在openlist中，需要检查代价是否可以更小
                    StateNodePtr node               = iterator->second;
                    double       new_traj_cost      = current_node_ptr->getTrajcost() + computeTrajCost(current_node_ptr, node);
                    double       new_heuristic_cost = computeHeuristicCost(node, goal_);
                    //如果新的代价更小则更新并加入openlist
                    if (node->getCost() > new_traj_cost + new_heuristic_cost) {
                        node->setTrajCost(new_traj_cost);
                        node->setHeuCost(new_heuristic_cost);
                        node->setDirection(FORWARD);
                        node->setStatus(IN_OPENSET);
                        node->setParentNode(current_node_ptr);
                        open_list_.emplace(node, node->getCost());
                    }
                }
            }
        }

        // for (int i = 0; i < serch_info_.steer_angle_segment_size; i++) {
        for (int i = 0; i < backward_projections_.size(); i++) {
            //获取所有合法的后方邻居节点
            if (getBackNeighbor(current_node_ptr, i, neighbor_node_ptr)) {
                std::unordered_map<int, GuidedHybridAStar::StateNodePtr>::iterator iterator =
                    graph_.find(neighbor_node_ptr->getIndex());
                if (iterator == graph_.end()) {
                    //说明是未访问过的节点
                    neighbor_node_ptr->setStatus(IN_OPENSET);
                    neighbor_node_ptr->setDirection(BACKWARD);
                    double traj_cost = current_node_ptr->getTrajcost() + computeTrajCost(current_node_ptr, neighbor_node_ptr);
                    neighbor_node_ptr->setTrajCost(traj_cost);
                    // neighbor_node_ptr->setHeuCost(computeHeuristicCost(neighbor_node_ptr, goal_));
                    double dist_cost = computeDistanceHeuristicCost(neighbor_node_ptr, goal_);
                    double obs_cost  = computeObstacleHeuristicCost(neighbor_node_ptr, goal_);
                    neighbor_node_ptr->setHeuCost(std::max(dist_cost, obs_cost));
                    // neighbor_node_ptr->setHeuCost(dist_cost);
                    neighbor_node_ptr->setParentNode(current_node_ptr);

                    // RCLCPP_INFO(logger_,
                    //             "x:%-12.5fy:%-12.5fheading:%-12.5fheu_cost:%-12.5ftraj_cost:%-12.5ftotal_cost:%-12.5fdist_"
                    //             "cost:%-12.5fobs_cost:%-12.5f",
                    //             neighbor_node_ptr->getState().x(),
                    //             neighbor_node_ptr->getState().y(),
                    //             neighbor_node_ptr->getState().z() * 180 / M_PI,
                    //             neighbor_node_ptr->getHeuCost(),
                    //             neighbor_node_ptr->getTrajcost(),
                    //             neighbor_node_ptr->getCost(),
                    //             dist_cost,
                    //             obs_cost);
                    if (visualization_tools_ptr_) {
                        visualization_tools_ptr_->add_point_pair(
                            Vec2d(costmap_->getOriginX() + (neighbor_node_ptr->getState().x() + 0.5) * costmap_->getResolution(),
                                  costmap_->getOriginY() + (neighbor_node_ptr->getState().y() + 0.5) * costmap_->getResolution()),
                            Vec2d(costmap_->getOriginX() + (current_node_ptr->getState().x() + 0.5) * costmap_->getResolution(),
                                  costmap_->getOriginY() + (current_node_ptr->getState().y() + 0.5) * costmap_->getResolution()));
                        visualization_tools_ptr_->publish_point_pairs();
                    }
                    graph_.emplace(neighbor_node_ptr->getIndex(), neighbor_node_ptr);
                    open_list_.emplace(neighbor_node_ptr, neighbor_node_ptr->getCost());
                }
                else if (iterator->second->getStatus() == IN_OPENSET) {
                    //说明已经在openlist中，需要检查代价是否可以更小
                    StateNodePtr node               = iterator->second;
                    double       new_traj_cost      = current_node_ptr->getTrajcost() + computeTrajCost(current_node_ptr, node);
                    double       new_heuristic_cost = computeHeuristicCost(node, goal_);
                    //如果新的代价更小则更新并加入openlist
                    if (node->getCost() > new_traj_cost + new_heuristic_cost) {
                        node->setTrajCost(new_traj_cost);
                        node->setHeuCost(new_heuristic_cost);
                        node->setDirection(BACKWARD);
                        node->setStatus(IN_OPENSET);
                        node->setParentNode(current_node_ptr);
                        open_list_.emplace(node, node->getCost());
                    }
                }
            }
        }
    }

    std::cout << "max_iterations:" << serch_info_.max_iterations << std::endl;
    std::cout << "iterations:" << iterations << std::endl;
    return false;
}

bool GuidedHybridAStar::backtracePath(VectorVec3d& path)
{
    std::cout << "backtracePath" << std::endl;
    StateNodePtr node = goal_;
    rclcpp::Rate r(2);
    while (node) {
        path.push_back(node->getState());

        std::cout << "backtracePath-->"
                  << "  x: " << node->getState().x() << "  y: " << node->getState().y() << std::endl;
        // r.sleep();
        node = node->getParentNode();
    }
    return true;
    // return false;
}


GuidedHybridAStar::~GuidedHybridAStar() {}


}   // namespace guided_hybrid_a_star