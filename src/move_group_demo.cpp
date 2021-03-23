#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  // 在CPU中开启一个线程
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt 使用JointModelGroup储存机械臂的joints，被称为PLANNING_GROUP.在整个运动中"planning group"和"joint model group"可以互换使用
  // 在官方提供的UR5e配置文件将其定义为两个move_group:{manipulator,endeffector}
  static const std::string PLANNING_GROUP = "manipulator";

  // 通过 创建planning_interface:`MoveGroupInterface` 类的实例可以轻松连接、控制或者计划planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // 通过 使用planning_scene_interface:`PlanningSceneInterface`类在virtual world添加或者移除障碍物
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // 获取机械臂状态
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // MoveItVisualTools包提供了很多在RVIZ下可用的可视化的物体、机器人和轨迹以及调试工具
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // 远程控制是一个在RVIZ通过按钮和键盘控制的工具
  visual_tools.loadRemoteControl();

  // RVIZ提供了很多类型的标记物，在这个例程中将会用到文本、圆柱体和球体
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing，批量发布用于减少发送到Rviz的消息数量，以实现大型可视化
  visual_tools.trigger();

  // 打印机器人参考坐标系的名称
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // 打印机器人在group的末端链接的名称
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // 我们可以得到机器人的所有groups
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // 开始例程
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  
  // 计划去一个目标位置
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // 我们可以计划一个在这个group的电机去获得末端的位姿
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  // 现在，我们打电话给planner来计算计划并将其可视化。 请注意，我们只是在计划，而不是要求move_group实际移动机器人。
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  
  // 我们也可以把这个计划在Rviz中用一条带有标记的线来进行可视化。
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


  /* Uncomment below line when working with a real robot */
  /* move_group.move(); */


  // 关节空间的运动规划。这将取代我们上面设置的姿态目标。
  // 首先，我们将创建一个引用当前机器人状态的指针。RobotState是包含所有当前位置/速度/加速度数据的对象。
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // 接下来获取组的当前关节值集合。
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // 现在，让我们修改其中一个关节，计划到新的关节空间目标并将计划可视化。
  joint_group_positions[0] = -1.0;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // 让整个规划过程可视化
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


  // 可以轻松地为机器人上的连杆指定路径约束。让我们为我们的组指定路径约束和姿势目标。首先定义路径约束。
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "ee_link";
  ocm.header.frame_id = "world";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // 现在，将其设置为组的路径约束。
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // 我们将重复使用我们已经计划的旧目标。 请注意，只有在当前状态已经满足路径约束的情况下，这才会起作用。（当前状态是符合约束的） 但是，我们也设置一个新的姿势。
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  // 按照新的起始位置和原来的目标位置进行规划
  move_group.setPoseTarget(target_pose1);

  // 用约束进行规划可能会很慢，因为每个样本必须调用一个反向运动学求解器。让我们从默认的5秒增加计划时间，以确保计划者有足够的时间成功。然后让规划过程可视化。
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // 在RVIZ中可视化显示
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // 当完成路径约束时，一定要清除约束。
  move_group.clearPathConstraints();


  // 可以通过指定末端路点（waypoint）列表直接规划笛卡尔路径。 请注意，我们从上面的新开始状态开始。 初始姿势（开始状态）不需要添加到航点（waypoint）列表，但添加它可以帮助进行可视化
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  // 通过每个关节的最大速度的缩放因子来降低机器人手臂的速度。 请注意，这不是最终效应器的速度。
  move_group.setMaxVelocityScalingFactor(0.1);

  // 我们希望以1厘米的分辨率插入笛卡尔路径，这就是为什么我们将0.01指定为笛卡儿平移中的最大步长。 （我们将跳转阈值指定为0.0，有效地禁用它。 ）——在操作真实硬件时禁用跳转阈值可能导致冗余连接发生大量不可预测的动作，这可能是一个安全问题
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // 在RVIZ中可视化显示
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 定义障碍物ROS消息
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // 物体的id用于识别它
  collision_object.id = "box1";

  // 定义一个box添加到世界
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  // 给box定义一个位姿
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // 现在，添加障碍物至世界中
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // 显示机器人状态
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // 等待MoveGroup接收并处理冲突对象消息
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // 现在，当我们计划一个轨道时，它将避开障碍物
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.w = 1.0;
  another_pose.position.x = 0.4;
  another_pose.position.y = -0.4;
  another_pose.position.z = 0.9;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // 在RVIZ中可视化显示
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // 现在，让我们把碰撞物体连接到机器人上。
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  // 显示机器人状态
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
                      "robot");

  // 现在，让我们从机器人上分离碰撞物体。
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  // 显示机器人状态
  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
                      "robot");

  // 现在，从世界中移除障碍物
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // 显示机器人状态
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");



  ros::shutdown();
  return 0;
}
