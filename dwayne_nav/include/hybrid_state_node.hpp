/*
 * @Author       : dwayne
 * @Date         : 2023-04-06
 * @LastEditTime : 2023-04-08
 * @Description  : hybrid A*中的状态节点
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include <Eigen/Dense>
#include <memory>
#include <type.hpp>

namespace guided_hybrid_a_star {
class StateNode
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    StateNode() = delete;

    explicit StateNode(const Vec3d& grid_index)
    {
        grid_index_  = grid_index;
        node_status_ = NOT_VISITED;
        parent_node_ = nullptr;
    }

    void reset()
    {
        node_status_ = NOT_VISITED;
        parent_node_ = nullptr;
    }

    void setHeuCost(double cost);
    void setTrajCost(double cost);


private:
    Vec3d                      state_;
    Vec3i                      grid_index_;
    std::shared_ptr<StateNode> parent_node_ = nullptr;
    NODE_STATUS                node_status_;
    double                     g_cost_;
    double                     f_cost_;
    double                     h_cost_;
    VectorVec3d                intermediate_states_;
};
}   // namespace guided_hybrid_a_star