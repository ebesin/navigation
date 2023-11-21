#pragma once

#include "vehicle_model_interface.h"

namespace VehicleModel {
class VehicleModelBicycleRearDriveFiveState : public VehicleModelInterface {
 private:
  /* data */
 public:
  VehicleModelBicycleRearDriveFiveState(const double& wheel_base);
  ~VehicleModelBicycleRearDriveFiveState() = default;

  Eigen::VectorXd getConstrainedU(const Eigen::VectorXd& u);

  void setCurState(
      const std::shared_ptr<VehicleStateInterface>& cur_state) override;
  void setEndState(
      const std::shared_ptr<VehicleStateInterface>& end_state) override;

  int getMatrixA(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                 const double dt, Eigen::MatrixXd& a) override;

  int getMatrixB(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                 const double dt, Eigen::MatrixXd& b) override;

  int toNextState(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                  const double dt, Eigen::VectorXd& next_state) override;
};
}  // namespace VehicleModel
