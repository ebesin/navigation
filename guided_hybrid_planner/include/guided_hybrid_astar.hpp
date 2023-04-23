/*
 * @Author       : dwayne
 * @Date         : 2023-04-08
 * @LastEditTime : 2023-04-22
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef GUIDED_HYBRID_A_STAR_HPP_
#define GUIDED_HYBRID_A_STAR_HPP_

#include "collision_checker.hpp"
#include "costmap_downsampler.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hybrid_state_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rs_path.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "visualization_tools.hpp"
#include <iomanip>
#include <iostream>
#include <vector>



namespace guided_hybrid_a_star {
class GuidedHybridAStar
{
public:
    typedef std::shared_ptr<StateNode>     StateNodePtr;
    typedef std::pair<float, unsigned int> ObstacleHeuristicElement;
    struct ObstacleHeuristicComparator
    {
        bool operator()(const ObstacleHeuristicElement& a, const ObstacleHeuristicElement& b) const { return a.first > b.first; }
    };

    typedef std::vector<ObstacleHeuristicElement> ObstacleHeuristicQueue;

    /**
     * @description  : constructor
     * @return        {*}
     */
    GuidedHybridAStar(nav2_costmap_2d::Costmap2D* costmap_ros, const SearchInfo& serch_info);

    /**
     * @description  : constructor
     * @return        {*}
     */
    GuidedHybridAStar(int width, int height, int dim3, const SearchInfo& serch_info);

    /**
     * @description  : destructor
     * @return        {*}
     */
    ~GuidedHybridAStar();


    /**
     * @description  : setCollisionChecker
     * @return        {*}
     */
    void setCollisionChecker(const std::shared_ptr<GridCollisionChecker>& checker) { this->collision_checker_ = checker; }

    void setSerchInfo(const SearchInfo& serch_info);

    /**
     * @description  : 进行hybrid a star计算
     * @return        {*}
     */
    bool complutePath();

    bool backtracePath(VectorVec3d& path);

    /**
     * @description  : 解析展开阶段，计算当前点到目标点是否有无碰撞的RS曲线
     * @param         {unsigned int} index:
     * @return        {*}
     */
    bool analyticExpansions(const StateNodePtr& current_node, const StateNodePtr& target_node, double& distance);


    /**
     * @description  : 判断是否越界
     * @param         {Vec2d} pos:
     * @return        {true--->越界  false---->未越界}
     */
    bool beyondBoundary(const Vec2d& pos) const;


    /**
     * @description  : 根据索引获取前方邻居节点
     * @param         {unsigned int} node_index:
     * @param         {unsigned int} neighbour_index:
     * @return        {*}
     */
    bool getFrontNeighbor(const StateNodePtr& current_node, unsigned int neighbour_index, StateNodePtr& next_node);

    /**
     * @description  : 根据索引获取后方邻居节点
     * @param         {unsigned int} node_index:
     * @param         {unsigned int} neighbour_index:
     * @return        {*}
     */
    bool getBackNeighbor(const StateNodePtr& current_node, unsigned int neighbour_index, StateNodePtr& next_node);


    /**
     * @description  : 设置起点
     * @param         {unsigned int} start:
     * @return        {*}
     */
    void setStart(const Vec3d& start_state);


    /**
     * @description  : 设置终点
     * @param         {unsigned int} goal:
     * @return        {*}
     */
    void setGoal(const Vec3d& goal_state);

    /**
     * @description  : 获取角度索引
     * @param         {double} angle:
     * @return        {*}
     */
    unsigned int getAngleIndex(const double heading) const;

    /**
     * @description  : 根据机器人状态计算对应节点索引
     * @param         {Vec3d&} state:
     * @return        {*}
     */
    unsigned int state2Index(const Vec3d& state) const;


    /**
     * @description  : 将航向角为0时机器人对应的运动原语计算并保存(后续计算任意角度的运动原语时只需做旋转变换)
     * @return        {*}
     */
    void initMotion();

    /**
     * @description  : 计算考虑障碍不考虑约束的启发式代价
     * @param         {StateNode&} current_node:
     * @return        {*}
     */
    double computeObstacleHeuristicCost(const StateNodePtr& current_node, const StateNodePtr& target_node);

    /**
     * @description  : 计算不考虑障碍考虑约束的启发式代价
     * @param         {StateNode&} current_node:
     * @return        {*}
     */
    double computeDistanceHeuristicCost(const StateNodePtr& current_node, const StateNodePtr& target_node);

    /**
     * @description  : 计算启发式代价
     * @param         {StateNode&} current_node:
     * @return        {*}
     */
    double computeHeuristicCost(const StateNodePtr& current_node, const StateNodePtr& target_node);

    /**
     * @description  : 重置关于计算考虑障碍启发式值得相关变量
     * @return        {*}
     */
    void resetObstacleHeuristic(nav2_costmap_2d::Costmap2D* costmap,
                                const StateNodePtr&         start_node,
                                const StateNodePtr&         goal_node);

    /**
     * @description  : 计算路径代价
     * @return        {*}
     */
    double computeTrajCost(const StateNodePtr& current_node_ptr, const StateNodePtr& neighbor_node_ptr);

    void resetAll();


    void setVisualizationToolsPtr(const std::shared_ptr<VisualizationTools>& visualization_tool)
    {
        visualization_tools_ptr_ = visualization_tool;
    }


private:
    rclcpp::Clock::SharedPtr                 clock_;
    rclcpp::Logger                           logger_{rclcpp::get_logger("GuidedHybridPlanner")};
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    nav2_costmap_2d::Costmap2D*              costmap_;
    std::shared_ptr<RSPath>                  rs_path_ptr_;
    std::shared_ptr<GridCollisionChecker>    collision_checker_;
    std::unique_ptr<CostmapDownsampler>      downsampler_;
    nav2_costmap_2d::Costmap2D*              sampled_costmap_;

    std::shared_ptr<VisualizationTools> visualization_tools_ptr_ = nullptr;

    //单位角度
    double       unit_angle_;
    unsigned int angle_size_;
    unsigned int x_size_;
    unsigned int y_size_;


    StateNodePtr start_;
    StateNodePtr goal_;

    SearchInfo              serch_info_;
    std::vector<MotionPose> forward_projections_;
    std::vector<MotionPose> backward_projections_;


    struct cmp
    {
        bool operator()(const std::pair<StateNodePtr, float> left, const std::pair<StateNodePtr, float> right) const
        {
            return left.second >= right.second;
        }
    };   // namespace guided_hybrid_a_star
    std::priority_queue<std::pair<StateNodePtr, float>, std::vector<std::pair<StateNodePtr, float>>, cmp> open_list_;
    std::unordered_map<int, StateNodePtr>                                                                 graph_;
    std::unordered_map<int, StateNodePtr>                                                                 close_list_;

    ObstacleHeuristicQueue obstacle_heuristic_queue_;
    std::vector<double>    obstacle_heuristic_lookup_table_;
};
}   // namespace guided_hybrid_a_star


#endif