#pragma once

#pragma once

#include <Eigen/Core>
#include <nav_msgs/msg/detail/odometry__struct.hpp>

#include "vehicle_state_interface.h"

using namespace VehicleState;

namespace VehicleModel {

class VehicleModelInterface {
 protected:
  const int m_dim_x;   //!< @brief dimension of state x
  const int m_dim_u;   //!< @brief dimension of input u
  double m_curvature;  //!< @brief curvature on the linearized point on path
  double m_wheelbase;  //!< @brief wheelbase of the vehicle [m]
  VehicleStateInterface m_cur_state;
  VehicleStateInterface m_end_state;
  Eigen::VectorXd m_cur_state_vec;
  Eigen::VectorXd m_end_state_vec;
  bool has_constrain{false};
  Eigen::VectorXd m_u_max;
  Eigen::VectorXd m_u_min;

 public:
  /**
   * @brief constructor
   * @param [in] dim_x dimension of state x
   * @param [in] dim_u dimension of input u
   * @param [in] wheelbase wheelbase of the vehicle [m]
   */
  VehicleModelInterface(int dim_x, int dim_u, double wheelbase);

  /**
   * @brief constructor
   * @param [in] dim_x dimension of state x
   * @param [in] dim_u dimension of input u
   */
  VehicleModelInterface(int dim_x, int dim_u);

  /**
   * @brief destructor
   */
  virtual ~VehicleModelInterface() = default;

  /**
   * @brief get state x dimension
   * @return state dimension
   */
  int getDimX();

  /**
   * @brief get input u dimension
   * @return input dimension
   */
  int getDimU();

  /**
   * @brief get wheelbase of the vehicle
   * @return wheelbase value [m]
   */
  double getWheelbase();

  /**
   * @brief set velocity
   * @param [in] velocity vehicle velocity
   */
  // void setVelocity(const double velocity);

  /**
   * @brief set control constrain
   * @param [in] u_min vehicle velocity
   * @param [in] u_max vehicle velocity
   */
  bool setConstrain(const Eigen::VectorXd& u_min, const Eigen::VectorXd& u_max);

  /**
   * @brief set curvature
   * @param [in] curvature curvature on the linearized point on path
   */
  void setCurvature(const double curvature);

  /**
   * @brief set current state
   * @param [in] cur_state vehicle current state
   */
  virtual void setCurState(const VehicleStateInterface& cur_state);

  /**
   * @brief get current state vector
   */
  Eigen::VectorXd getCurStateVec();

  VehicleStateInterface getCurState();

  /**
   * @brief set current state
   * @param [in] cur_state vehicle current state
   */
  virtual void setEndState(const VehicleStateInterface& end_state);

  /**
   * @brief get current state vector
   */
  Eigen::VectorXd getEndStateVec();

  VehicleStateInterface getEndState();

  /**
   * @brief calculate discrete model matrix of x_k+1 = a_d * xk + b_d * uk
   * @param [out] a_d coefficient matrix
   * @param [out] b_d coefficient matrix
   * @param [in] dt Discretization time [s]
   */
  // virtual void calculateDiscreteMatrix(Eigen::MatrixXd& a_d,
  //                                      Eigen::MatrixXd& b_d,
  //                                      const double dt) = 0;

  /**
   * @brief calculate matrix a
   * @param [in] x state matrix
   * @param [in] u control matrix
   * @param [in] dt Discretization time [s]
   * @param [oyt] matrix a
   */
  virtual int getMatrixA(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                         const double dt, Eigen::MatrixXd& a) = 0;

  /**
   * @brief calculate matrix b
   * @param [in] x state matrix
   * @param [in] u control matrix
   * @param [in] dt Discretization time [s]
   * @param [oyt] matrix b
   */
  virtual int getMatrixB(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                         const double dt, Eigen::MatrixXd& b) = 0;

  /**
   * @brief calculate the next state based on the current state and control
   * @param [in] x state matrix
   * @param [in] u control matrix
   * @param [in] dt Discretization time [s]
   * @param [oyt] next state
   */
  virtual int toNextState(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                          const double dt, Eigen::VectorXd& next_state) = 0;

  /**
   * @brief calculate reference input
   * @param [out] u_ref input
   */
  // virtual void calculateReferenceInput(Eigen::MatrixXd& u_ref) = 0;
};
}  // namespace VehicleModel
