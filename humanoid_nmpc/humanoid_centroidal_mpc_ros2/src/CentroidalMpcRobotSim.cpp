/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <pthread.h>
#include <sched.h>

#include <ocs2_sqp/SqpMpc.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <motorium_drake/DrakeDriver.h>
#include <motorium_hal/RobotHAL.h>
#include <motorium_mujoco/MujocoDriver.h>

#include <humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h>
#include <humanoid_centroidal_mpc/mrt/CentroidalMpcMrtJointController.h>
#include "humanoid_common_mpc_ros2/ros_comm/Ros2ProceduralMpcMotionManager.h"
#include "humanoid_common_mpc_ros2/visualization/HumanoidVisualizer.h"

using namespace ocs2;
using namespace ocs2::humanoid;

namespace {

// Passive joint damping of both simulators (Nm s/rad). A gains file's dampingCompensation should match it.
constexpr double kSimJointDamping = 10.0;

// Adds the simulator driver selected by the ROS parameter "simulator" ("mujoco" or "drake").
// MuJoCo simulates the scene XML; Drake simulates the URDF the HAL was created from, resolving its
// package:// URIs (meshes) through the ROS parameter "model_package". Drake applies initialActionPtr
// until the first action arrives; MuJoCo has no such option.
void addSimulatorDriver(motorium::hal::RobotHAL& robotHal,
                        rclcpp::Node& node,
                        const std::string& mjxFile,
                        const std::shared_ptr<const motorium::model::RobotState>& initStatePtr,
                        const std::shared_ptr<const motorium::model::RobotJointFeedbackAction>& initialActionPtr) {
  const std::string simulator = node.declare_parameter<std::string>("simulator", "mujoco");
  const std::string modelPackage = node.declare_parameter<std::string>("model_package", "");

  if (simulator == "mujoco") {
    motorium::mujoco::MujocoSimConfig config;
    config.scenePath = mjxFile;
    config.verbose = true;
    config.defaultJointDamping = kSimJointDamping;
    config.initStatePtr_ = initStatePtr;
    robotHal.addDriver<motorium::mujoco::MujocoDriver>(config);
  } else if (simulator == "drake") {
    motorium::drake_sim::DrakeSimConfig config;
    config.initStatePtr = initStatePtr;
    config.initialActionPtr = initialActionPtr;
    config.defaultJointDamping = kSimJointDamping;
    config.headless = false;  // Meshcat URL is printed on startup
    config.verbose = true;
    if (!modelPackage.empty()) {
      config.packagePaths = {{modelPackage, ament_index_cpp::get_package_share_directory(modelPackage)}};
    }
    robotHal.addDriver<motorium::drake_sim::DrakeDriver>(config);
  } else {
    throw std::runtime_error("Unknown simulator '" + simulator + "'; expected 'mujoco' or 'drake'.");
  }
  std::cerr << "Simulator: " << simulator << std::endl;
}

// Blocks until the MPC policy keeps the base within kHeightTolerance of initialBaseHeight at its
// start and end (the legacy drake_mrt_sim_node check: >= 0.78 m for the G1's 0.7925 m), or throws
// after kTimeout.
void waitForStandingPolicy(const CentroidalMpcMrtJointController& controller, double initialBaseHeight) {
  constexpr double kHeightTolerance = 0.0125;  // [m]
  constexpr auto kTimeout = std::chrono::seconds(15);

  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  // A NaN height (no policy yet) fails the comparison, so this also waits for the first policy.
  while (!(controller.policyMinBaseHeight() >= initialBaseHeight - kHeightTolerance)) {
    if (std::chrono::steady_clock::now() > deadline) {
      throw std::runtime_error("MPC warm-up failed: the policy base height " + std::to_string(controller.policyMinBaseHeight()) +
                               " m did not reach " + std::to_string(initialBaseHeight - kHeightTolerance) + " m.");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cerr << "MPC warm-up done: policy base height " << controller.policyMinBaseHeight() << " m." << std::endl;
}

// Restricts the calling thread to the CPUs in the integer-array ROS parameter `parameter`. Threads
// created afterwards inherit this set, which is how the solver/MPC threads and the physics/MRT
// threads end up on separate cores. An empty list leaves the affinity unchanged.
void pinCurrentThreadToCpus(const std::vector<int64_t>& cpus, const std::string& parameter) {
  if (cpus.empty()) return;
  cpu_set_t cpuSet;
  CPU_ZERO(&cpuSet);
  for (const int64_t cpu : cpus) {
    CPU_SET(static_cast<int>(cpu), &cpuSet);
  }
  if (pthread_setaffinity_np(pthread_self(), sizeof(cpuSet), &cpuSet) != 0) {
    throw std::runtime_error("Failed to apply CPU affinity from parameter '" + parameter + "'.");
  }
  std::cerr << parameter << ":";
  for (const int64_t cpu : cpus) std::cerr << " " << cpu;
  std::cerr << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> programArgs;
  programArgs = rclcpp::remove_ros_arguments(argc, argv);
  if (programArgs.size() < 6) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }

  const std::string robotName(argv[1]);
  const std::string taskFile(argv[2]);
  const std::string referenceFile(argv[3]);
  const std::string urdfFile(argv[4]);
  const std::string gaitFile(argv[5]);
  const std::string mjxFile(argv[6]);

  rclcpp::init(argc, argv);

  // Launch MPC ROS node
  rclcpp::Node::SharedPtr nodeHandle = std::make_shared<rclcpp::Node>(robotName + "_centroidal_mpc");

  // Optional core split: the MPC (solver thread pool, MPC thread) runs on "mpc_cpus", the simulator
  // physics thread and the MRT loop on "sim_cpus". Pinned before the solver creates its threads.
  const auto mpcCpus = nodeHandle->declare_parameter<std::vector<int64_t>>("mpc_cpus", std::vector<int64_t>{});
  const auto simCpus = nodeHandle->declare_parameter<std::vector<int64_t>>("sim_cpus", std::vector<int64_t>{});
  pinCurrentThreadToCpus(mpcCpus, "mpc_cpus");

  // Robot interface
  CentroidalMpcInterface interface(taskFile, urdfFile, referenceFile);

  // MPC
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());

  auto qos = rclcpp::QoS(1);
  qos.best_effort();

  std::shared_ptr<HumanoidVisualizer> humanoidVisualizer(
      new HumanoidVisualizer(taskFile, interface.getPinocchioInterface(), interface.getMpcRobotModel(), nodeHandle));

  // Reference and motion management for Procedural MPC
  CentroidalMpcTargetTrajectoriesCalculator mpcTargetTrajectoriesCalculator(
      referenceFile, interface.getMpcRobotModel(), interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
      interface.mpcSettings().timeHorizon_);
  ProceduralMpcMotionManager::VelocityTargetToTargetTrajectories targetTrajectoriesFunc =
      [&mpcTargetTrajectoriesCalculator](const vector4_t& velocityTarget, scalar_t initTime, scalar_t finalTime,
                                         const vector_t& initState) mutable {
        return mpcTargetTrajectoriesCalculator.commandedVelocityToTargetTrajectories(velocityTarget, initTime, initState);
      };
  auto ros2ProceduralMpcMotionManager = std::make_shared<Ros2ProceduralMpcMotionManager>(
      gaitFile, referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(), targetTrajectoriesFunc);

  ros2ProceduralMpcMotionManager->subscribe(nodeHandle, qos);

  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  mpc.getSolverPtr()->addSynchronizedModule(ros2ProceduralMpcMotionManager);

  // Init Sim state
  motorium::model::RobotDescription robotDescription(urdfFile);
  auto mutableInitStatePtr = std::make_shared<motorium::model::RobotState>(robotDescription, 2);
  mutableInitStatePtr->setConfigurationToZero();

  const vector_t& initMpcState = interface.getInitialState();
  const auto& mpcModel = interface.getMpcRobotModel();
  mutableInitStatePtr->setRootPositionInWorldFrame(mpcModel.getBasePosition(initMpcState));

  vector_t mpcJointAngles = mpcModel.getJointAngles(initMpcState);
  // Todo set non zero orientation;
  std::vector<motorium::joint_index_t> mpcJointIndices = robotDescription.getJointIndices(interface.modelSettings().mpcModelJointNames);
  for (size_t i = 0; i < mpcJointIndices.size(); i++) {
    mutableInitStatePtr->setJointPosition(mpcJointIndices[i], mpcJointAngles[i]);
  }

  std::cerr << "initState: " << mutableInitStatePtr->getRootPositionInWorldFrame().transpose() << std::endl;

  // Freeze the initial state so it can be shared safely between the sim driver and the MPC.
  // Moving out of the mutable pointer leaves it null, so only const access remains.
  const std::shared_ptr<const motorium::model::RobotState> initStatePtr = std::move(mutableInitStatePtr);

  // The HAL provides the robot description now; the simulator driver is added once the MPC is warm.
  motorium::hal::RobotHAL robotHal(urdfFile);

  CentroidalMpcMrtJointController mpcJointController(robotHal.getRobotDescription(), interface.modelSettings(),
                                                     interface.getMpcRobotModel(), mpc, interface.getPinocchioInterface(),
                                                     interface.mpcSettings().mpcDesiredFrequency_, humanoidVisualizer);

  // Optional per-joint gains; without a file the controller keeps its defaults.
  const std::string jointGainsFile = nodeHandle->declare_parameter<std::string>("joint_gains_file", "");
  if (!jointGainsFile.empty()) {
    const auto& jointNames = robotHal.getRobotDescription().getJointNames();
    const std::vector<JointGains> jointGains = loadJointGains(jointGainsFile, jointNames);
    std::cerr << "Joint gains from " << jointGainsFile << ":\n";
    for (size_t i = 0; i < jointNames.size(); ++i) {
      std::cerr << "  " << jointNames[i] << ": kp " << jointGains[i].kp << ", kd " << jointGains[i].kd << ", damping compensation "
                << jointGains[i].dampingCompensation << "\n";
    }
    mpcJointController.setJointGains(jointGains);
  }

  std::cout << "MPC MRT joint controller is set up. " << std::endl;

  // size_t mrtDeltaTMicroSeconds_ = 1000000 / (interface.mpcSettings().mrtDesiredFrequency_);
  size_t mrtDeltaTMicroSeconds_ = 1000000 / (500);
  motorium::model::RobotState state(robotHal.getRobotDescription(), 2);
  motorium::model::RobotJointFeedbackAction action(robotHal.getRobotDescription());

  // Warm-up: the MPC solves on the frozen initial state until its policy holds the initial base
  // height at both ends, so the robot is never released onto a policy that is still settling.
  mpcJointController.startMpcThread(*initStatePtr);
  waitForStandingPolicy(mpcJointController, initStatePtr->getRootPositionInWorldFrame().z());

  // The first MPC action (PD around the policy plus feed-forward torques) holds the robot from the
  // first physics step until the MRT loop sends its own.
  const auto initialActionPtr = std::make_shared<motorium::model::RobotJointFeedbackAction>(robotHal.getRobotDescription());
  mpcJointController.computeJointControlAction(0.0, *initStatePtr, *initStatePtr, *initialActionPtr);

  addSimulatorDriver(robotHal, *nodeHandle, mjxFile, initStatePtr, initialActionPtr);

  // From here on this thread runs the MRT loop; the simulator's physics thread inherits the set.
  pinCurrentThreadToCpus(simCpus, "sim_cpus");
  robotHal.startDrivers();

  rclcpp::spin_some(nodeHandle);

  while (true) {
    auto targetTimeForNextIteration = std::chrono::steady_clock::now() + std::chrono::microseconds(mrtDeltaTMicroSeconds_);

    // Read the latest state, compute and send the action right away to keep the state-to-action latency minimal.
    robotHal.updateState(state);
    mpcJointController.computeJointControlAction(0.0, state, state, action);
    robotHal.setAction(action);

    rclcpp::spin_some(nodeHandle);

    auto currentTime = std::chrono::steady_clock::now();
    if (currentTime > targetTimeForNextIteration) {
      auto delay = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - targetTimeForNextIteration).count();

      std::cerr << "Warning: MRT loop running slow by " << delay << " microseconds." << std::endl;
    } else {
      // Sleep in case sim loop is faster than specified
      std::this_thread::sleep_until(targetTimeForNextIteration);
    }
  }

  std::cout << "ende..." << std::endl;

  return 0;
}
