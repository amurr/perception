// moveit_master: the ONLY node that commands the robot through MoveGroupInterface.
// It exposes one action, "moveit_master/execute_sequence", which runs a list of
// SubMovements in order: arm pose / arm named state / straight-line (cartesian) arm move /
// gripper named state / gripper joint value / wait / attach / detach / allow collision.
//
// Bimanual: every step names the arm it runs on (SubMovement.arm, "" = default_arm). Each arm
// has its own MoveGroupInterface for the arm and the gripper group, its TCP link and its
// gripper links. Parameters (per arm, "{arm}" is replaced by the arm name):
//
//   arms                      [left, right]
//   default_arm               left
//   arm_group_format          {arm}_arm                 (override per arm: <arm>.arm_group)
//   gripper_group_format      {arm}_gripper             (<arm>.gripper_group, "" = no gripper)
//   tcp_link_format           {arm}_robotiq_85_tcp      (<arm>.tcp_link, "" = SRDF end effector)
//   gripper_root_link_format  {arm}_robotiq_85_base_link (<arm>.gripper_root_link)
//   planning_time             5.0
//   planning_attempts         5
//   plan_retries              3   (plan again when a plan fails, e.g. the validated path
//                                  clips an object: OMPL is random, the next try usually works)
//
// The sequence and all its parameters come from articulation_orchestrator. Steps run one at a
// time (one arm moves, the other stands still and is a collision object for the planner).
// If a sequence fails or is canceled, the PlanningScene changes it made (attached objects,
// allowed collisions) are rolled back, so the scene again matches what the orchestrator expects.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

#include <moveit_master/action/execute_sequence.hpp>
#include <moveit_master/msg/sub_movement.hpp>

using ExecuteSequence = moveit_master::action::ExecuteSequence;
using SubMovement = moveit_master::msg::SubMovement;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteSequence>;
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;
using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;
using moveit::core::MoveItErrorCode;

// Everything moveit_master needs to drive one arm + its gripper.
struct Arm
{
  std::string name;
  std::unique_ptr<MoveGroupInterface> arm;
  std::unique_ptr<MoveGroupInterface> gripper;    // null: arm without gripper group
  std::string eef_link;                           // TCP: pose / cartesian targets, attach link
  std::vector<std::string> gripper_links;         // may touch a grasped object
};

class MoveItMaster
{
public:
  explicit MoveItMaster(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    const auto names = node_->get_parameter_or<std::vector<std::string>>(
      "arms", std::vector<std::string>{"left", "right"});
    for (const auto & name : names) {
      arms_.emplace(name, makeArm(name));
    }
    if (arms_.empty()) {
      throw std::runtime_error("parameter 'arms' is empty");
    }
    plan_retries_ = std::max(1, node_->get_parameter_or<int>("plan_retries", 3));
    default_arm_ = node_->get_parameter_or<std::string>("default_arm", names.front());
    if (arms_.count(default_arm_) == 0) {
      throw std::runtime_error("default_arm '" + default_arm_ + "' not in 'arms'");
    }

    scene_client_ = node_->create_client<GetPlanningScene>("/get_planning_scene");

    server_ = rclcpp_action::create_server<ExecuteSequence>(
      node_, "moveit_master/execute_sequence",
      // accept only one goal at a time
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteSequence::Goal>) {
        return busy_.exchange(true) ? rclcpp_action::GoalResponse::REJECT
                                    : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // cancel: stop every arm and gripper; the step loop exits afterwards
      [this](std::shared_ptr<GoalHandle>) {
        for (auto & [_, a] : arms_) {
          a.arm->stop();
          if (a.gripper) {
            a.gripper->stop();
          }
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // run in a separate thread so the executor keeps spinning
      [this](std::shared_ptr<GoalHandle> gh) {
        std::thread([this, gh] { execute(gh); }).detach();
      });

    RCLCPP_INFO(node_->get_logger(), "moveit_master ready, %zu arm(s), default '%s'",
                arms_.size(), default_arm_.c_str());
  }

private:
  // "<arm>.<key>" if set, else `format` with "{arm}" replaced by the arm name.
  std::string armParam(const std::string & arm, const std::string & key, const std::string & format)
  {
    std::string value;
    if (node_->get_parameter(arm + "." + key, value)) {
      return value;
    }
    value = node_->get_parameter_or<std::string>(key + "_format", format);
    for (auto pos = value.find("{arm}"); pos != std::string::npos; pos = value.find("{arm}")) {
      value.replace(pos, 5, arm);
    }
    return value;
  }

  Arm makeArm(const std::string & name)
  {
    Arm a;
    a.name = name;
    a.arm = std::make_unique<MoveGroupInterface>(node_, armParam(name, "arm_group", "{arm}_arm"));
    a.arm->setPlanningTime(node_->get_parameter_or<double>("planning_time", 5.0));
    a.arm->setNumPlanningAttempts(node_->get_parameter_or<int>("planning_attempts", 5));

    const auto gripper_group = armParam(name, "gripper_group", "{arm}_gripper");
    if (!gripper_group.empty()) {
      a.gripper = std::make_unique<MoveGroupInterface>(node_, gripper_group);
      // gripper moves are short: full speed within the joint limits (not the sequence scaling)
      a.gripper->setMaxVelocityScalingFactor(1.0);
      a.gripper->setMaxAccelerationScalingFactor(1.0);
    }

    // TCP: pose / cartesian targets and attached objects refer to this link.
    a.eef_link = armParam(name, "tcp_link", "{arm}_robotiq_85_tcp");
    if (a.eef_link.empty()) {
      a.eef_link = a.arm->getEndEffectorLink();
    }
    if (a.eef_link.empty()) {
      a.eef_link = a.arm->getLinkNames().back();
    }
    if (a.arm->getRobotModel()->getLinkModel(a.eef_link) == nullptr) {
      throw std::runtime_error("tcp_link '" + a.eef_link + "' of arm '" + name + "' not in the robot model");
    }
    a.arm->setEndEffectorLink(a.eef_link);

    // Gripper mount + every link below it (body, knuckles, fingers, TCP).
    const auto root = armParam(name, "gripper_root_link", "{arm}_robotiq_85_base_link");
    const auto * root_model = a.arm->getRobotModel()->getLinkModel(root);
    if (root_model == nullptr) {
      throw std::runtime_error("gripper_root_link '" + root + "' of arm '" + name + "' not in the robot model");
    }
    addLinksBelow(root_model, a.gripper_links);
    RCLCPP_INFO(node_->get_logger(), "arm '%s': group '%s', gripper '%s', tcp '%s', %zu gripper links",
                name.c_str(), a.arm->getName().c_str(), gripper_group.c_str(), a.eef_link.c_str(),
                a.gripper_links.size());
    return a;
  }

  static void addLinksBelow(const moveit::core::LinkModel * link, std::vector<std::string> & out)
  {
    out.push_back(link->getName());
    for (const auto * joint : link->getChildJointModels()) {
      addLinksBelow(joint->getChildLinkModel(), out);
    }
  }

  Arm * findArm(const std::string & name)
  {
    auto it = arms_.find(name.empty() ? default_arm_ : name);
    return it == arms_.end() ? nullptr : &it->second;
  }

  // Plan + execute whatever target is currently set on `group`; planning is retried.
  MoveItErrorCode planAndExecute(MoveGroupInterface & group)
  {
    MoveGroupInterface::Plan plan;
    MoveItErrorCode ec = MoveItErrorCode::PLANNING_FAILED;
    for (int i = 0; i < plan_retries_ && ec != MoveItErrorCode::SUCCESS; ++i) {
      if (i > 0) {
        RCLCPP_WARN(node_->get_logger(), "planning '%s' failed (%s), retry %d/%d", group.getName().c_str(),
                    moveit::core::errorCodeToString(ec).c_str(), i, plan_retries_ - 1);
      }
      ec = group.plan(plan);
    }
    if (ec == MoveItErrorCode::SUCCESS) {
      ec = group.execute(plan);
    }
    return ec;
  }

  // Straight-line TCP move by `offset`, collision-checked. frame "" = planning frame,
  // "tcp" = axes of the TCP link at the start of the move (orientation is kept).
  MoveItErrorCode moveCartesian(Arm & a, const geometry_msgs::msg::Vector3 & offset, const std::string & frame)
  {
    auto & g = *a.arm;
    Eigen::Vector3d d(offset.x, offset.y, offset.z);
    if (frame == "tcp") {
      d = g.getCurrentState()->getGlobalLinkTransform(a.eef_link).rotation() * d;
    } else if (!frame.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "cartesian frame '%s' unknown (\"\" or \"tcp\")", frame.c_str());
      return MoveItErrorCode::INVALID_GOAL_CONSTRAINTS;
    }
    auto target = g.getCurrentPose(a.eef_link).pose;
    target.position.x += d.x();
    target.position.y += d.y();
    target.position.z += d.z();

    moveit_msgs::msg::RobotTrajectory traj;
    const double fraction = g.computeCartesianPath({target}, 0.005, traj);
    if (fraction < 0.999) {
      RCLCPP_ERROR(node_->get_logger(), "cartesian path only %.0f%% feasible", fraction * 100.0);
      return MoveItErrorCode::PLANNING_FAILED;
    }
    // computeCartesianPath times the path at full speed -> re-time it with the sequence scaling
    robot_trajectory::RobotTrajectory rt(g.getRobotModel(), g.getName());
    rt.setRobotTrajectoryMsg(*g.getCurrentState(), traj);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(rt, vel_, vel_)) {
      return MoveItErrorCode::INVALID_MOTION_PLAN;
    }
    rt.getRobotTrajectoryMsg(traj);
    return g.execute(traj);
  }

  // Attach a world object to the arm's TCP link (attach=true) or put it back into the world.
  MoveItErrorCode attach(const Arm & a, const std::string & id, bool attach)
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = a.eef_link;
    aco.object.id = id;
    aco.object.operation = attach ? moveit_msgs::msg::CollisionObject::ADD
                                  : moveit_msgs::msg::CollisionObject::REMOVE;
    if (attach) {
      aco.touch_links = a.gripper_links;
    }
    if (!psi_.applyAttachedCollisionObject(aco)) {
      return MoveItErrorCode::INVALID_OBJECT_NAME;
    }
    if (attach) {
      attached_[id] = a.name;
    } else {
      attached_.erase(id);
    }
    return MoveItErrorCode::SUCCESS;
  }

  // Allow (or undo allowing) contacts between object `id` and `names`.
  MoveItErrorCode setAllowed(const std::string & id, const std::vector<std::string> & names, bool allow)
  {
    if (!scene_client_->wait_for_service(std::chrono::seconds(2))) {
      return MoveItErrorCode::TIMED_OUT;
    }
    auto req = std::make_shared<GetPlanningScene::Request>();
    req->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = scene_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      return MoveItErrorCode::TIMED_OUT;
    }

    // the ACM in a scene diff replaces the whole matrix -> modify the current one
    collision_detection::AllowedCollisionMatrix acm(future.get()->scene.allowed_collision_matrix);
    for (const auto & name : names) {
      if (allow) {
        acm.setEntry(id, name, true);
        allowed_[id].insert(name);
      } else {
        acm.removeEntry(id, name);
        allowed_[id].erase(name);
      }
    }
    if (allowed_[id].empty()) {
      allowed_.erase(id);
    }

    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    acm.getMessage(diff.allowed_collision_matrix);
    return psi_.applyPlanningScene(diff) ? MoveItErrorCode::SUCCESS : MoveItErrorCode::FAILURE;
  }

  // `id` may touch the arm's gripper links + `extra` (e.g. the table under it).
  MoveItErrorCode allowCollision(const Arm & a, const std::string & id, std::vector<std::string> extra, bool allow)
  {
    extra.insert(extra.end(), a.gripper_links.begin(), a.gripper_links.end());
    return setAllowed(id, extra, allow);
  }

  // Undo scene changes of a failed / canceled sequence.
  void rollbackScene()
  {
    for (const auto & [id, arm] : std::map<std::string, std::string>(attached_)) {
      attach(arms_.at(arm), id, false);
    }
    for (const auto & [id, names] : std::map<std::string, std::set<std::string>>(allowed_)) {
      setAllowed(id, {names.begin(), names.end()}, false);
    }
  }

  MoveItErrorCode runStep(const SubMovement & step)
  {
    if (step.type == SubMovement::WAIT) {
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(step.duration * 1000)));
      return MoveItErrorCode::SUCCESS;
    }
    Arm * a = findArm(step.arm);
    if (a == nullptr) {
      RCLCPP_ERROR(node_->get_logger(), "unknown arm '%s'", step.arm.c_str());
      return MoveItErrorCode::INVALID_GROUP_NAME;
    }
    const bool gripper_step = step.type == SubMovement::GRIPPER_NAMED || step.type == SubMovement::GRIPPER_JOINT;
    if (gripper_step && !a->gripper) {
      RCLCPP_ERROR(node_->get_logger(), "arm '%s' has no gripper group", a->name.c_str());
      return MoveItErrorCode::INVALID_GROUP_NAME;
    }
    switch (step.type) {
      case SubMovement::ARM_POSE:
        a->arm->setPoseTarget(step.pose, a->eef_link);
        return planAndExecute(*a->arm);
      case SubMovement::ARM_NAMED:
        if (!a->arm->setNamedTarget(step.named_target)) {
          return MoveItErrorCode::INVALID_GOAL_CONSTRAINTS;
        }
        return planAndExecute(*a->arm);
      case SubMovement::ARM_CARTESIAN:
        return moveCartesian(*a, step.offset, step.frame);
      case SubMovement::GRIPPER_NAMED:
        if (!a->gripper->setNamedTarget(step.named_target)) {
          return MoveItErrorCode::INVALID_GOAL_CONSTRAINTS;
        }
        return planAndExecute(*a->gripper);
      case SubMovement::GRIPPER_JOINT:
        if (!a->gripper->setJointValueTarget(a->gripper->getActiveJoints().front(), step.joint_value)) {
          return MoveItErrorCode::INVALID_GOAL_CONSTRAINTS;
        }
        return planAndExecute(*a->gripper);
      case SubMovement::ATTACH:
        return attach(*a, step.object_id, true);
      case SubMovement::DETACH:
        return attach(*a, step.object_id, false);
      case SubMovement::ALLOW_COLLISION:
        return allowCollision(*a, step.object_id, step.touch_links, step.allow);
      default:
        return MoveItErrorCode::INVALID_MOTION_PLAN;
    }
  }

  void execute(std::shared_ptr<GoalHandle> gh)
  {
    const auto goal = gh->get_goal();
    auto result = std::make_shared<ExecuteSequence::Result>();
    auto feedback = std::make_shared<ExecuteSequence::Feedback>();

    vel_ = goal->velocity_scaling > 0.0 ? goal->velocity_scaling : 0.2;
    for (auto & [_, a] : arms_) {
      a.arm->setMaxVelocityScalingFactor(vel_);
      a.arm->setMaxAccelerationScalingFactor(vel_);
    }
    attached_.clear();
    allowed_.clear();

    result->success = true;
    for (size_t i = 0; i < goal->steps.size(); ++i) {
      const auto & step = goal->steps[i];
      if (gh->is_canceling()) {
        result->success = false;
        result->failed_step = i;
        result->message = "canceled";
        break;
      }
      feedback->current_step = i;
      feedback->current_label = step.label;
      gh->publish_feedback(feedback);

      const auto ec = runStep(step);
      if (ec != MoveItErrorCode::SUCCESS) {
        result->success = false;
        result->failed_step = i;
        result->message = moveit::core::errorCodeToString(ec) + " (" + step.arm + ": " + step.label + ")";
        RCLCPP_ERROR(node_->get_logger(), "step %zu failed: %s", i, result->message.c_str());
        break;
      }
    }
    for (auto & [_, a] : arms_) {
      a.arm->clearPoseTargets();
    }
    if (!result->success) {
      rollbackScene();
    }

    if (gh->is_canceling()) {
      gh->canceled(result);
    } else if (result->success) {
      result->message = "done";
      gh->succeed(result);
    } else {
      gh->abort(result);
    }
    busy_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  std::map<std::string, Arm> arms_;
  std::string default_arm_;
  PlanningSceneInterface psi_;
  rclcpp::Client<GetPlanningScene>::SharedPtr scene_client_;
  rclcpp_action::Server<ExecuteSequence>::SharedPtr server_;
  std::atomic<bool> busy_{false};

  int plan_retries_{3};
  double vel_{0.2};
  // scene changes made by the running sequence (rolled back if it fails)
  std::map<std::string, std::string> attached_;                 // object id -> arm
  std::map<std::string, std::set<std::string>> allowed_;        // object id -> allowed names
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "moveit_master",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // MultiThreaded: MoveGroupInterface (joint states) and the action server must keep spinning
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor] { executor.spin(); });

  MoveItMaster master(node);

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
