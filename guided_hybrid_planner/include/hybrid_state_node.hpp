/*
    * @Author       : dwayne
    * @Date         : 2023-04-06
 * @LastEditTime : 2023-04-23
    * @Description  : hybrid A*中的状态节点
    * 
    * Copyright (c) 2023 by dwayne, All Rights Reserved. 
    */

#ifndef HYBRID_STATE_NODE_HPP_
#define HYBRID_STATE_NODE_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <type.hpp>

namespace guided_hybrid_a_star {
class StateNode
{

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    StateNode()
    {
        node_status_ = NOT_VISITED;
        parent_node_ = nullptr;
        direction_   = NO;
    };

    /**
     * @description  : 使用索引初始化节点
     * @param         {int} index:
     * @return        {*}
     */
    explicit StateNode(const unsigned int index)
    {
        this->index_ = index;
        direction_   = NO;
        node_status_ = NOT_VISITED;
        parent_node_ = nullptr;
    }

    void reset()
    {
        node_status_ = NOT_VISITED;
        parent_node_ = nullptr;
        direction_   = NO;
    }

    void        setStatus(const NODE_STATUS status) { this->node_status_ = status; }
    NODE_STATUS getStatus() const { return this->node_status_; }

    void      setDirection(const DIRECTION direction) { this->direction_ = direction; }
    DIRECTION getDirection() const { return this->direction_; }

    void         setIndex(const unsigned int index) { this->index_ = index; }
    unsigned int getIndex() const { return this->index_; }

    void setState(const Vec3d& state)
    {
        this->state_ = state;
        if (this->state_.z() < 0) this->state_.z() += 2 * M_PI;
        if (this->state_.z() > 2 * M_PI) this->state_.z() -= 2 * M_PI;
    }
    Vec3d getState() const { return this->state_; }

    void                       setParentNode(std::shared_ptr<StateNode> node) { this->parent_node_ = node; }
    std::shared_ptr<StateNode> getParentNode() { return this->parent_node_; }

    void   setHeuCost(double cost) { this->h_cost_ = cost; }
    double getHeuCost() const { return h_cost_; }

    double getTrajcost() const { return g_cost_; }
    void   setTrajCost(double cost) { this->g_cost_ = cost; }

    double getCost() const { return g_cost_ + h_cost_; }

    void setIntermediateStates(VectorVec3d intermediate_states) { this->intermediate_states_ = intermediate_states; }
    void getIntermediateStates(VectorVec3d& states) const { states = intermediate_states_; }


private:
    unsigned int               index_;
    Vec3d                      state_;
    std::shared_ptr<StateNode> parent_node_ = nullptr;
    NODE_STATUS                node_status_;
    DIRECTION                  direction_;
    double                     g_cost_;
    double                     h_cost_;
    VectorVec3d                intermediate_states_;
};
}   // namespace guided_hybrid_a_star

#endif