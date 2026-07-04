#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

struct MoveRequest
{
  int id;
  geometry_msgs::msg::Pose approach;
  std::vector<double> direction;
  double distance;
};

class CycleController
{
public:
  explicit CycleController(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    move_group_(node_, "ur_manipulator"),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    move_group_.setPoseReferenceFrame("base_link");
    move_group_.setEndEffectorLink("ee_link");
    move_group_.setPlanningTime(5.0);
    move_group_.setNumPlanningAttempts(10);
    move_group_.setMaxVelocityScalingFactor(0.25);
    move_group_.setMaxAccelerationScalingFactor(0.25);

    auto start_qos = rclcpp::QoS(1)
      .reliable()
      .transient_local();

    start_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/start",
      start_qos,
      std::bind(&CycleController::startCallback, this, std::placeholders::_1)
    );

    auto report_qos = rclcpp::QoS(10).best_effort();

    tcp_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/tcp_pose", report_qos);
    cycle_status_pub_ = node_->create_publisher<std_msgs::msg::String>("/cycle_status", report_qos);
    motion_state_pub_ = node_->create_publisher<std_msgs::msg::String>("/motion_state", report_qos);
    attempted_count_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/attempted_count", report_qos);
    completed_count_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/completed_count", report_qos);

    publish_timer_ = node_->create_wall_timer(
      50ms,
      std::bind(&CycleController::publishReports, this)
    );

    setCycleStatus("ready");
    setMotionState("idle");

    RCLCPP_INFO(node_->get_logger(), "BCR cycle controller ready.");
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cycle_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr attempted_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr completed_count_pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex state_mutex_;
  std::string cycle_status_ = "ready";
  std::string motion_state_ = "idle";
  int attempted_count_ = 0;
  int completed_count_ = 0;
  bool running_ = false;

  void setCycleStatus(const std::string& value)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cycle_status_ = value;
  }

  void setMotionState(const std::string& value)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    motion_state_ = value;
  }

  void setCounts(int attempted, int completed)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    attempted_count_ = attempted;
    completed_count_ = completed;
  }

  void publishReports()
  {
    std::string cycle_status;
    std::string motion_state;
    int attempted;
    int completed;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      cycle_status = cycle_status_;
      motion_state = motion_state_;
      attempted = attempted_count_;
      completed = completed_count_;
    }

    std_msgs::msg::String status_msg;
    status_msg.data = cycle_status;
    cycle_status_pub_->publish(status_msg);

    std_msgs::msg::String motion_msg;
    motion_msg.data = motion_state;
    motion_state_pub_->publish(motion_msg);

    std_msgs::msg::Int32 attempted_msg;
    attempted_msg.data = attempted;
    attempted_count_pub_->publish(attempted_msg);

    std_msgs::msg::Int32 completed_msg;
    completed_msg.data = completed;
    completed_count_pub_->publish(completed_msg);

    try
    {
      auto tf = tf_buffer_.lookupTransform(
        "base_link",
        "ee_link",
        tf2::TimePointZero
      );

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = node_->now();
      pose.header.frame_id = "base_link";
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;

      tcp_pose_pub_->publish(pose);
    }
    catch (const std::exception&)
    {
      // TF may not be ready during startup. Keep publishing other reports.
    }
  }

  void startCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (running_)
      {
        RCLCPP_WARN(node_->get_logger(), "Cycle already running. Ignoring /start.");
        return;
      }
      running_ = true;
    }

    std::string payload = msg->data;

    std::thread([this, payload]() {
      runCycle(payload);

      std::lock_guard<std::mutex> lock(state_mutex_);
      running_ = false;
    }).detach();
  }

  std::vector<MoveRequest> parseMoves(const std::string& payload)
  {
    auto root = json::parse(payload);

    if (!root.contains("frame_id") || root["frame_id"] != "base_link")
    {
      throw std::runtime_error("JSON frame_id must be base_link");
    }

    std::vector<MoveRequest> moves;

    for (const auto& item : root["moves"])
    {
      MoveRequest m;
      m.id = item["id"].get<int>();

      auto p = item["approach"]["position"];
      auto q = item["approach"]["orientation"];

      m.approach.position.x = p[0].get<double>();
      m.approach.position.y = p[1].get<double>();
      m.approach.position.z = p[2].get<double>();

      m.approach.orientation.x = q[0].get<double>();
      m.approach.orientation.y = q[1].get<double>();
      m.approach.orientation.z = q[2].get<double>();
      m.approach.orientation.w = q[3].get<double>();

      auto d = item["linear"]["direction"];
      m.direction = {
        d[0].get<double>(),
        d[1].get<double>(),
        d[2].get<double>()
      };

      m.distance = item["linear"]["distance"].get<double>();

      moves.push_back(m);
    }

    return moves;
  }

  bool executePoseTarget(const geometry_msgs::msg::Pose& pose)
  {
    move_group_.setPoseTarget(pose, "ee_link");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_.plan(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Pose plan failed.");
      move_group_.clearPoseTargets();
      return false;
    }

    auto exec_result = move_group_.execute(plan);
    move_group_.clearPoseTargets();

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Pose execution failed.");
      return false;
    }

    return true;
  }

  bool executeCartesian(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& end)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start);
    waypoints.push_back(end);

    moveit_msgs::msg::RobotTrajectory trajectory;

    const double eef_step = 0.002;
    const double jump_threshold = 0.0;

    double fraction = move_group_.computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory
    );

    if (fraction < 0.95)
    {
      RCLCPP_ERROR(node_->get_logger(), "Cartesian path failed. Fraction: %.3f", fraction);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;

    auto exec_result = move_group_.execute(cart_plan);

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Cartesian execution failed.");
      return false;
    }

    return true;
  }

  geometry_msgs::msg::Pose makeLinearEndPose(const MoveRequest& move)
  {
    geometry_msgs::msg::Pose end = move.approach;

    double dx = move.direction[0];
    double dy = move.direction[1];
    double dz = move.direction[2];

    double norm = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (norm < 1e-9)
    {
      throw std::runtime_error("Linear direction has zero length");
    }

    dx /= norm;
    dy /= norm;
    dz /= norm;

    end.position.x += dx * move.distance;
    end.position.y += dy * move.distance;
    end.position.z += dz * move.distance;

    return end;
  }

  bool returnHome()
  {
    setMotionState("approach");
    move_group_.setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_.plan(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Home plan failed.");
      setMotionState("idle");
      return false;
    }

    auto exec_result = move_group_.execute(plan);
    setMotionState("idle");

    return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  void runCycle(const std::string& payload)
  {
    setCycleStatus("running");
    setMotionState("idle");
    setCounts(0, 0);

    int attempted = 0;
    int completed = 0;

    try
    {
      auto moves = parseMoves(payload);

      returnHome();

      for (const auto& move : moves)
      {
        attempted++;
        setCounts(attempted, completed);

        RCLCPP_INFO(node_->get_logger(), "Starting move id=%d", move.id);

        bool success = true;

        setMotionState("approach");
        success = executePoseTarget(move.approach);

        geometry_msgs::msg::Pose end_pose;

        if (success)
        {
          end_pose = makeLinearEndPose(move);

          setMotionState("linear");
          success = executeCartesian(move.approach, end_pose);
        }

        if (success)
        {
          setMotionState("retract");
          success = executeCartesian(end_pose, move.approach);
        }

        setMotionState("idle");

        if (success)
        {
          completed++;
          setCounts(attempted, completed);
          RCLCPP_INFO(node_->get_logger(), "Move id=%d completed.", move.id);
        }
        else
        {
          RCLCPP_WARN(node_->get_logger(), "Move id=%d failed. Skipping to next.", move.id);
        }
      }

      returnHome();
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Cycle error: %s", e.what());
    }

    setMotionState("idle");
    setCycleStatus("completed");

    RCLCPP_INFO(node_->get_logger(), "Cycle completed.");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("bcr_cycle_controller", options);

  auto controller = std::make_shared<CycleController>(node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
