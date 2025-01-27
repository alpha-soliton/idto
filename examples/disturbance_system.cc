#include "examples/disturbance_system.h"

#include <random>
#include <iostream>

namespace idto {
namespace examples {

using Eigen::VectorXd;
using Eigen::Vector3;

using drake::math::RigidTransform;
using drake::multibody::ExternallyAppliedSpatialForce;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialForce;
using drake::multibody::RigidBody;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;
using drake::systems::VectorBase;
using drake::systems::PublishEvent;
using drake::systems::TriggerType;

DisturbanceGenerator::DisturbanceGenerator(const MultibodyPlant<double>* plant,
                                   const double force_mag, const double period)
    : plant_(plant), force_mag_(force_mag), period_(period),
      gen(std::random_device{}()), dis(-force_mag, force_mag) {
  box_body_index_ = plant->GetBodyByName("box").index();
  this->DeclareAbstractOutputPort(
      "spatial_forces",
      &DisturbanceGenerator::CalcDisturbance);
  body_poses_port_index_ = this->DeclareAbstractInputPort(
      "body_poses",
      // ポート型 = std::vector<RigidTransform<double>>
      drake::Value<std::vector<RigidTransform<double>>>{}
  ).get_index();
  this->DeclareAbstractOutputPort(
      "target_transform",
      &DisturbanceGenerator::OutputTargetTransform);
  this->DeclarePeriodicDiscreteUpdateEvent(period_, 0.0,
      &DisturbanceGenerator::PerStep);
  this->DeclareDiscreteState(2); // fx, fy
}

void DisturbanceGenerator::CalcDisturbance(const Context<double>& context,
    std::vector<ExternallyAppliedSpatialForce<double>>* output) const {
  output->resize(2);

  // x direction.
  const RigidBody<double>* box_body_x = &plant_->GetBodyByName("box_ghost_body_x");
  (*output)[0].body_index = box_body_x->index();
  (*output)[0].p_BoBq_B = box_body_x->default_com();
  auto random_fx = context.get_discrete_state(0).value()[0];
  const Vector3<double> tau_x(0.0, 0.0, 0.0);
  const Vector3<double> force_x(random_fx, 0.0, 0.0);
  auto random_force_x = SpatialForce<double>(tau_x, force_x);
  (*output)[0].F_Bq_W = random_force_x;

  // y direction.
  const RigidBody<double>* box_body_y = &plant_->GetBodyByName("box_ghost_body_y");
  (*output)[1].body_index = box_body_y->index();
  (*output)[1].p_BoBq_B = box_body_y->default_com();
  auto random_fy = context.get_discrete_state(0).value()[1];
  const Vector3<double> tau_y(0.0, 0.0, 0.0);
  const Vector3<double> force_y(0.0, random_fy, 0.0);
  auto random_force_y = SpatialForce<double>(tau_y, force_y);
  (*output)[1].F_Bq_W = random_force_y;

  // Clear force.
  // TODO(manabun):
  //auto random_f = context.get_mutable_discrete_state().get_mutable_value(0);
  auto current_time = context.get_time();
  if ((current_time - last_force_update) > 0.1) {
    auto random_f = const_cast<drake::systems::Context<double>&>(context).get_mutable_discrete_state().get_mutable_value(0);
    random_f(0) = 0.0;
    random_f(1) = 0.0;
  }
}

void DisturbanceGenerator::OutputTargetTransform(const Context<double>& context,
    RigidTransform<double>* output) const {

  const auto& poses_value =
      this->get_input_port(body_poses_port_index_)
          .template Eval<std::vector<RigidTransform<double>>>(context);

  // box ボディのワールド変換
  const RigidTransform<double>& X_WBox = poses_value[box_body_index_];

  /*
  // Get box transform.
  const RigidBody<double>* box_body_x = &plant_->GetBodyByName("box");
  const RigidTransform<double>& box_x_X_WB = box_body_x->EvalPoseInWorld(context);
  (*output) = box_x_X_WB;
  */
  (*output) = X_WBox;
}

EventStatus DisturbanceGenerator::PerStep(const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto random_fx = discrete_state->get_mutable_value(0);
  double random_fx_value = generate_random_force();
  random_fx(0) = random_fx_value;

  auto random_fy = discrete_state->get_mutable_value(0);
  double random_fy_value = generate_random_force();
  random_fy(1) = random_fy_value;

  last_force_update = context.get_time();

  return EventStatus::Succeeded();
}

} // namespace examples
} // namespace idto
