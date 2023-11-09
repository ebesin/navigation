#include "vehicle_model_interface.h"

namespace VehicleModel {
VehicleModelInterface::VehicleModelInterface(int dim_x, int dim_u,
                                             double wheelbase)
    : m_dim_x(dim_x), m_dim_u(dim_u), m_wheelbase(wheelbase) {}

VehicleModelInterface::VehicleModelInterface(int dim_x, int dim_u)
    : m_dim_x(dim_x), m_dim_u(dim_u), m_wheelbase(0) {}

int VehicleModelInterface::getDimX() { return m_dim_x; }

int VehicleModelInterface::getDimU() { return m_dim_u; }

double VehicleModelInterface::getWheelbase() { return m_wheelbase; }

void VehicleModelInterface::setCurvature(double curvature) {
  m_curvature = curvature;
}

Eigen::VectorXd VehicleModelInterface::getCurStateVec() {
  return m_cur_state_vec;
}

Eigen::VectorXd VehicleModelInterface::getEndStateVec() {
  return m_end_state_vec;
}

VehicleStateInterface VehicleModelInterface::getCurState() {
  return m_cur_state;
}

VehicleStateInterface VehicleModelInterface::getEndState() {
  return m_end_state;
}

void VehicleModelInterface::setCurState(
    const VehicleStateInterface& cur_state) {
  m_cur_state = cur_state;
}

void VehicleModelInterface::setEndState(
    const VehicleStateInterface& end_state) {
  m_end_state = end_state;
}

bool VehicleModelInterface::setConstrain(const Eigen::VectorXd& u_min,
                                         const Eigen::VectorXd& u_max) {
  has_constrain = false;
  if (u_min.size() != m_dim_u || u_max.size() != m_dim_u) return false;
  has_constrain = true;
  m_u_min = u_min;
  m_u_max = u_max;
  return true;
}

}  // namespace VehicleModel
