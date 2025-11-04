#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <thread>
#include <chrono>
#include <Eigen/Geometry>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>


int main(int argc, char **argv)
{
  // Inicializar ROS2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hello_moveit");
  auto logger = rclcpp::get_logger("hello_moveit");

  // Executor en hilo separado
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Interfaz de MoveGroup
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group_interface(node, "manipulator");
  move_group_interface.setPlanningTime(5.0);

  // Herramienta de visualización
  moveit_visual_tools::MoveItVisualTools visual_tools(
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC);
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // === Lambdas para visualización ===
  auto draw_title = [&visual_tools](const std::string &text) {
    auto text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                             rviz_visual_tools::XLARGE);
    visual_tools.trigger();
  };

  auto prompt = [&visual_tools](const std::string &text) {
    visual_tools.prompt(text);
  };

  auto draw_trajectory_tool_path =
      [&visual_tools,
       jmg = move_group_interface.getRobotModel()->getJointModelGroup("manipulator")](
          const moveit_msgs::msg::RobotTrajectory &trajectory) {
        visual_tools.publishTrajectoryLine(trajectory, jmg);
        visual_tools.trigger();
      };

  // pose objetivo
auto const target_pose = [] {
  geometry_msgs::msg::Pose msg;
  msg.orientation.y = 0.8;
  msg.orientation.w = 0.6;
  msg.position.x = 0.1;
  msg.position.y = 0.4;
  msg.position.z = 0.4;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// objeto de colision 
auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box1";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.2;
  box_pose.position.y = 0.2;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();


  // === Planificación ===
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  planning_scene_interface.applyCollisionObject(collision_object);
  bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    draw_title("Planning");
    draw_trajectory_tool_path(plan.trajectory);
    prompt("Press 'Next' in RViz to execute");
    draw_title("Executing");
    move_group_interface.execute(plan);
  }
  else
  {
    draw_title("Planning Failed!");
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Apagado limpio
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
