#include "examples/example_base.h"
#include <drake/common/find_resource.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/prismatic_joint.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");
DEFINE_string(trajectory, "/home/manabu-nishiura/planning_through_contact/tree_planar_pushing_intersegment_refined_qa_qu_xy.yaml",
              "GQDP trajectory file used to feed initial guess of IDTO");
DEFINE_double(nominal_update_dt, 0.4,
              "Interval for updating the nominal_state.");
DEFINE_string(yaml_file, "/home/manabu-nishiura/idto/examples/simple_maze/simple_maze_gqdp.yaml",
              "Configuration file containing the definition of the trajectory "
              "optimization problem");
DEFINE_bool(time_varying_cost, false,
            "When set to true, the trajectory optimization process takes into "
            "account costs that vary over time, allowing for a more dynamic "
            "and responsive optimization. If set to false, the optimization "
            "assumes constant costs throughout the trajectory, leading to a "
            "usual optimization approach.");


struct SavedTrajectory {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(x_trj));
  }

  std::vector<Eigen::VectorXd> x_trj;
};

std::vector<Eigen::VectorXd> ExtractTrajectory(
    const std::vector<Eigen::VectorXd>& trajectory, int num_joints) {
  std::vector<Eigen::VectorXd> desired_trajectory;
  for (const Eigen::VectorXd& step : trajectory) {
    desired_trajectory.push_back(step.head(num_joints));
  }
  return desired_trajectory;
}

std::vector<Eigen::VectorXd> ExtractInitialTrajectory(
    const std::vector<Eigen::VectorXd>& trajectory, int num_joints,
    int num_steps) {
  std::vector<Eigen::VectorXd> desired_trajectory;
  int traj_length = 0;
  for (const Eigen::VectorXd& step : trajectory) {
    if (traj_length < num_steps) {
        desired_trajectory.push_back(step.head(num_joints));
        traj_length += 1;
    }
    else {break;}
  }
  return desired_trajectory;
}

using idto::examples::TrajOptExampleParams;
using drake::yaml::LoadYamlFile;
using drake::yaml::LoadYamlOptions;

namespace idto {
namespace examples {
namespace hopper {

using drake::geometry::Box;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;

/**
 * A simple maze.
 */
class SimpleMazeExample : public TrajOptExample {
 public:
  SimpleMazeExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(0.5, -2.0, 0.5);
    const Vector3d target_pose(0.0, 0.0, 0.0);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);

    // Add a hopper
    std::string urdf_file =
          "/home/manabu-nishiura/idto/examples/models/simple_maze.sdf";
    //    idto::FindIdtoResourceOrThrow("idto/examples/models/simple_maze.sdf");
    Parser(plant).AddModels(urdf_file);

    // Add collision with the ground
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -5.5));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground,
                                  Box(25, 25, 10), "ground", green);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 10), "ground",
                                     CoulombFriction<double>(0.5, 0.5));
  }
};

}  // namespace hopper
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load the problem definition from YAML
  TrajOptExampleParams default_options;
  TrajOptExampleParams options =
      LoadYamlFile<TrajOptExampleParams>(FLAGS_yaml_file, {}, default_options);

  LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  yaml_options.allow_cpp_with_no_yaml = true;
  SavedTrajectory gqdp_trajectory_;
  gqdp_trajectory_ =
    LoadYamlFile<SavedTrajectory>(FLAGS_trajectory, {}, {}, yaml_options);
  std::vector<Eigen::VectorXd> initial_trajectory;
  std::vector<Eigen::VectorXd> whole_trajectory;
  const int num_underactuated_dof = 3;
  const int num_actuated_dof = 2;
  whole_trajectory = ExtractTrajectory(gqdp_trajectory_.x_trj,
                                        num_underactuated_dof+num_actuated_dof);
  initial_trajectory = ExtractInitialTrajectory(gqdp_trajectory_.x_trj,
                                        num_underactuated_dof+num_actuated_dof,
                                        options.num_steps + 1);

  idto::examples::hopper::SimpleMazeExample example;
  example.RunExample("idto/examples/simple_maze/simple_maze_gqdp.yaml",
      initial_trajectory, whole_trajectory, FLAGS_nominal_update_dt,
      FLAGS_test, FLAGS_time_varying_cost);

  return 0;
}
