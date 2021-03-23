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
  target_pose1.position.x = -0.08;
  target_pose1.position.y = 0.5;
  target_pose1.position.z = 0.03;
  move_group.setPoseTarget(target_pose1);

  // 现在，我们打电话给planner来计算计划并将其可视化。 请注意，我们只是在计划，而不是要求move_group实际移动机器人。
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  bool success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  
  // 我们也可以把这个计划在Rviz中用一条带有标记的线来进行可视化。
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  ros::shutdown();
  return 0;
}
