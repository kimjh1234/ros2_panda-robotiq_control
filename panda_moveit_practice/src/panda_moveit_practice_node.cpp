// Ex - Kinematics in MoveIt, Panda
//  Reference) https://moveit.picknik.ai/main/doc/examples/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#define D2R M_PI/180.0

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("panda_moveit_practice_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("panda_moveit_practice_node", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "panda_arm";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Setup the planner
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;
  move_group.setPoseReferenceFrame("panda_link0");
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);
  move_group.setPlanningTime(10.0);


  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "move_group_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Kinematics_in_MoveIt_Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();


  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the example
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start demo");


  // Step 1) Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // Create an pointer that references the current robot's state. RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr robot_state = move_group.getCurrentState(); // Here, update the robot's state by using GetCurrentState

  // Get the joint names for the raw pointer 'joint_model_group'
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // We can retrieve the current set of joint values stored in the state for the robot arm.
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "[Step 1] Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }


  // Step 2) Enforcing the Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  /* Set one joint in the robot arm outside its joint limit */
  joint_values[0] = 5.57;
  robot_state->setJointGroupPositions(joint_model_group, joint_values);

  /* Check whether any joint is outside its joint limits */
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "[Step 2] (Set one joint outsiede its joint limit) Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  RCLCPP_INFO_STREAM(LOGGER, "[Step 2] Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  robot_state->enforceBounds();
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "[Step 2] (After enforcing the joint limit) Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  RCLCPP_INFO_STREAM(LOGGER, "[Step 2] Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));


  // Step 3) Planning to a joint-space goal after enforcing the joint limits
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  move_group.setJointValueTarget(joint_values);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "[Step 3] Visualizing plan 2 (joint-space goal after enforcing the joint limits) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz:
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "After_enforcing_the_joint_limits", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("[Step 3] Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Moving to a joint-space goal after enforcing the joint limits
  move_group.execute(my_plan);


  // Step 4) Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  robot_state = move_group.getCurrentState(); // Here, update the robot's state by using GetCurrentState
  const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform(move_group.getEndEffectorLink().c_str());

  /* Print end-effector pose. Remember that this is in the model frame */
  RCLCPP_INFO_STREAM(LOGGER, "[Step 4] Translation: \n" << end_effector_state.translation() << "\n");
  RCLCPP_INFO_STREAM(LOGGER, "[Step 4] Rotation: \n" << end_effector_state.rotation() << "\n");


  // Step 5) Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // We can now solve inverse kinematics (IK) for the robot.
  // To solve IK, we will need the following:
  //
  //  * The desired pose of the end-effector
  //  * The timeout: 0.1 s

  // Create a copy of end_effector_state and modify the translation
  Eigen::Isometry3d modified_end_effector_state = end_effector_state;
  modified_end_effector_state.translation() = Eigen::Vector3d(-0.1, 0.4, 0.3);  // Modify translation
  modified_end_effector_state.linear() = end_effector_state.rotation();  // Keep the original rotation

  double timeout = 0.1;
  bool found_ik = robot_state->setFromIK(joint_model_group, modified_end_effector_state, timeout);

  // Create a Pose message and set the position and orientation from modified_end_effector_state
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = modified_end_effector_state.translation().x();
  target_pose.position.y = modified_end_effector_state.translation().y();
  target_pose.position.z = modified_end_effector_state.translation().z();

  // Convert Eigen rotation (3x3 matrix) to quaternion
  Eigen::Quaterniond quat(modified_end_effector_state.rotation());
  target_pose.orientation.x = quat.x();
  target_pose.orientation.y = quat.y();
  target_pose.orientation.z = quat.z();
  target_pose.orientation.w = quat.w();

  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose, "IK_solution", rvt::LARGE, rvt::WHITE);
  visual_tools.trigger();
  visual_tools.prompt("[Step 5] Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(LOGGER, "[Step 5] Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // Planning to the IK solution
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    move_group.setJointValueTarget(joint_values);

    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "[Step 5] Visualizing plan 2 (IK solution) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz:
    visual_tools.publishText(text_pose, "Joint_Space_Goal_from_IK_solution", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("[Step 5] Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Moving to a joint-space goal
    move_group.execute(my_plan);

    // Forward Kinematics
    // ^^^^^^^^^^^^^^^^^^
    robot_state = move_group.getCurrentState(); // Here, update the robot's state by using GetCurrentState
    const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform(move_group.getEndEffectorLink().c_str());

    /* Print end-effector pose. Remember that this is in the model frame */
    RCLCPP_INFO_STREAM(LOGGER, "[Step 5] Translation: \n" << end_effector_state.translation() << "\n");
    RCLCPP_INFO_STREAM(LOGGER, "[Step 5] Rotation: \n" << end_effector_state.rotation() << "\n");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "[Step 5] Did not find IK solution");
  }


  // Step 6) Get the Jacobian
  // ^^^^^^^^^^^^^^^^^^
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                          reference_point_position, jacobian);
  RCLCPP_INFO_STREAM(LOGGER, "[Step 6] Jacobian: \n" << jacobian << "\n");


  // END_TUTORIAL
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}
