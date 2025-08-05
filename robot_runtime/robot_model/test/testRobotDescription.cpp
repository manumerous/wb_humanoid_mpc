
#include "robot_model/RobotDescription.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>

#include "absl/container/flat_hash_map.h"
#include "robot_model/RobotDescription.h"

namespace robot::model {
namespace testing {

class RobotDescriptionTest : public ::testing::Test {
 protected:
  // Create a temporary URDF file for testing
  void SetUp() override {
    tempDir_ = std::filesystem::temp_directory_path() / "robot_model_test";
    std::filesystem::create_directories(tempDir_);
    urdf_path_ = tempDir_ / "test_robot.urdf";

    // Create a simple test URDF with a few joints
    std::ofstream urdfFile(urdf_path_);
    urdfFile << R"(<?xml version="1.0"?>
        <robot name="test_robot">
            <link name="base_link"/>
            <link name="shoulder_link"/>
            <link name="elbow_link"/>
            <link name="wrist_link"/>
            <link name="hand_link"/>
            
            <joint name="shoulder_joint" type="revolute">
                <parent link="base_link"/>
                <child link="shoulder_link"/>
                <axis xyz="0 0 1"/>
                <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
            </joint>
            
            <joint name="elbow_joint" type="revolute">
                <parent link="shoulder_link"/>
                <child link="elbow_link"/>
                <axis xyz="0 1 0"/>
                <limit lower="-2.0" upper="2.0" effort="80" velocity="1.5"/>
            </joint>
            
            <joint name="wrist_joint" type="revolute">
                <parent link="elbow_link"/>
                <child link="wrist_link"/>
                <axis xyz="1 0 0"/>
                <limit lower="-1.0" upper="1.0" effort="50" velocity="3.0"/>
            </joint>
            
            <joint name="fixed_joint" type="fixed">
                <parent link="base_link"/>
                <child link="hand_link"/>
            </joint>
        </robot>)";
    urdfFile.close();
  }

  void TearDown() override { std::filesystem::remove_all(tempDir_); }

  std::filesystem::path tempDir_;
  std::filesystem::path urdf_path_;
};

// Test constructor with valid URDF
TEST_F(RobotDescriptionTest, Constructor) {
  ASSERT_NO_THROW({ RobotDescription robotDesc(urdf_path_.string()); });

  RobotDescription robotDesc(urdf_path_.string());
  EXPECT_EQ(robotDesc.getURDFPath(), urdf_path_.string());
  EXPECT_EQ(robotDesc.getNumJoints(),
            3);  // Should only count the revolute joints
}

// Test constructor with non-existent URDF
TEST_F(RobotDescriptionTest, ConstructorWithNonexistentFile) {
  std::string nonexistentPath = tempDir_ / "nonexistent.urdf";
  EXPECT_THROW({ RobotDescription robotDesc(nonexistentPath); }, std::runtime_error);
}

// Test constructor with invalid URDF content
TEST_F(RobotDescriptionTest, ConstructorWithInvalidURDF) {
  std::filesystem::path invalidPath = tempDir_ / "invalid.urdf";
  std::ofstream invalidFile(invalidPath);
  invalidFile << "This is not a valid URDF file";
  invalidFile.close();

  EXPECT_THROW({ RobotDescription robotDesc(invalidPath.string()); }, std::runtime_error);
}

// Test getURDFName method
TEST_F(RobotDescriptionTest, GetURDFName) {
  RobotDescription robotDesc(urdf_path_.string());
  EXPECT_EQ(robotDesc.getURDFName(), "test_robot.urdf");
}

// Test containsJoint method
TEST_F(RobotDescriptionTest, ContainsJoint) {
  RobotDescription robotDesc(urdf_path_.string());

  EXPECT_TRUE(robotDesc.containsJoint("shoulder_joint"));
  EXPECT_TRUE(robotDesc.containsJoint("elbow_joint"));
  EXPECT_TRUE(robotDesc.containsJoint("wrist_joint"));
  EXPECT_FALSE(robotDesc.containsJoint("fixed_joint"));  // Fixed joints should be excluded
  EXPECT_FALSE(robotDesc.containsJoint("nonexistent_joint"));
}

// Test getJointDescription method
TEST_F(RobotDescriptionTest, GetJointDescription) {
  RobotDescription robotDesc(urdf_path_.string());

  // Test valid joint
  const JointDescription& shoulderDesc = robotDesc.getJointDescription("shoulder_joint");
  EXPECT_EQ(shoulderDesc.min_angle, -1.57);
  EXPECT_EQ(shoulderDesc.max_angle, 1.57);
  EXPECT_EQ(shoulderDesc.max_effort, 100.0);
  EXPECT_EQ(shoulderDesc.max_velocity, 2.0);

  // Test another valid joint
  const JointDescription& elbowDesc = robotDesc.getJointDescription("elbow_joint");
  EXPECT_EQ(elbowDesc.min_angle, -2.0);
  EXPECT_EQ(elbowDesc.max_angle, 2.0);
  EXPECT_EQ(elbowDesc.max_effort, 80.0);
  EXPECT_EQ(elbowDesc.max_velocity, 1.5);

  // Test nonexistent joint should throw
  EXPECT_THROW({ robotDesc.getJointDescription("nonexistent_joint"); }, std::out_of_range);
}

// Test getJointIndex method
TEST_F(RobotDescriptionTest, GetJointIndex) {
  RobotDescription robotDesc(urdf_path_.string());

  int32_t shoulderIndex = robotDesc.getJointIndex("shoulder_joint");
  int32_t elbowIndex = robotDesc.getJointIndex("elbow_joint");
  int32_t wristIndex = robotDesc.getJointIndex("wrist_joint");

  // Each joint should have a unique index
  EXPECT_NE(shoulderIndex, elbowIndex);
  EXPECT_NE(shoulderIndex, wristIndex);
  EXPECT_NE(elbowIndex, wristIndex);

  // Indices should be within expected range (0 to numJoints-1)
  EXPECT_GE(shoulderIndex, 0);
  EXPECT_LT(shoulderIndex, 3);

  // Test nonexistent joint should throw
  EXPECT_THROW({ robotDesc.getJointIndex("nonexistent_joint"); }, std::out_of_range);
}

// Test getJointName method
TEST_F(RobotDescriptionTest, GetJointName) {
  RobotDescription robotDesc(urdf_path_.string());

  // Get indices first
  int32_t shoulderIndex = robotDesc.getJointIndex("shoulder_joint");
  int32_t elbowIndex = robotDesc.getJointIndex("elbow_joint");
  int32_t wristIndex = robotDesc.getJointIndex("wrist_joint");

  // Test mapping from index back to name
  EXPECT_EQ(robotDesc.getJointName(shoulderIndex), "shoulder_joint");
  EXPECT_EQ(robotDesc.getJointName(elbowIndex), "elbow_joint");
  EXPECT_EQ(robotDesc.getJointName(wristIndex), "wrist_joint");

  // Test invalid index should throw
  EXPECT_THROW({ robotDesc.getJointName(999); }, std::out_of_range);
}

// Test operator<< for JointDescription
TEST_F(RobotDescriptionTest, JointDescriptionStreamOperator) {
  JointDescription jointDesc;
  jointDesc.id = 42;
  jointDesc.min_angle = -1.5;
  jointDesc.max_angle = 1.5;
  jointDesc.max_velocity = 2.0;
  jointDesc.max_effort = 100.0;

  std::stringstream ss;
  ss << jointDesc;

  // Expected format may vary depending on your implementation
  // This is just a simple check that something was written
  EXPECT_FALSE(ss.str().empty());

  // If your implementation has a specific format, test it here
  // For example:
  // EXPECT_EQ(ss.str(), "JointDescription(id=42, min_angle=-1.5,
  // max_angle=1.5, max_velocity=2.0, max_effort=100.0)");
}

// Test operator<< for RobotDescription
TEST_F(RobotDescriptionTest, RobotDescriptionStreamOperator) {
  RobotDescription robotDesc(urdf_path_.string());

  std::stringstream ss;
  ss << robotDesc;

  // Expected format may vary depending on your implementation
  // This is just a simple check that something was written
  EXPECT_FALSE(ss.str().empty());

  // Basic checks that the output contains important information
  std::string output = ss.str();
  EXPECT_TRUE(output.find("test_robot.urdf") != std::string::npos);
  EXPECT_TRUE(output.find("shoulder_joint") != std::string::npos);
  EXPECT_TRUE(output.find("elbow_joint") != std::string::npos);
  EXPECT_TRUE(output.find("wrist_joint") != std::string::npos);
}

}  // namespace testing
}  // namespace robot::model

// Main function that runs all tests
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}