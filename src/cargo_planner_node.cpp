/**
 * @file cargo_planner_node.cpp
 * @brief ROS 2 node implementation.
 */

#include "cargo_planner/cargo_planner_node.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "cargo_planner/grid_map_utils.hpp"
#include "cargo_planner/greedy_placer.hpp"
#include "cargo_planner/visualization.hpp"

namespace cargo_planner
{

namespace
{

constexpr char kContainerOccupancyGridRegistrationService[] = "container_occupancy_grid_registration";
constexpr char kCargoListRegistrationService[]              = "cargo_list_registration";
constexpr char kPlanCargoAction[]                           = "cargo_planning";
constexpr char kPlacementMarkersTopic[]                     = "cargo_markers";
constexpr char kFreeMapTopic[]                              = "container_free_map";

}  // namespace

// ── Constructor ───────────────────────────────────────────────────────────────

CargoPlacerNode::CargoPlacerNode(const rclcpp::NodeOptions& options):
  Node("cargo_planner", options),
  planner_(std::make_unique<GreedyPlacer>())
{
  // ── Declare parameters ──────────────────────────────────────────────────
  occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
  pallet_margin_      = declare_parameter<double>("pallet_margin", 0.05);
  enable_rotation_    = declare_parameter<bool>("enable_rotation", true);

  // ── Rebuild planner config whenever parameters arrive ──────────────────
  CargoPlanner::Config cfg;
  cfg.occupied_threshold = occupied_threshold_;
  cfg.pallet_margin      = pallet_margin_;
  cfg.enable_rotation    = enable_rotation_;
  planner_               = CargoPlanner(std::make_unique<GreedyPlacer>(), cfg);

  // ── Services ────────────────────────────────────────────────────────────
  srv_container_occupancy_grid_registration_ = create_service<
    cargo_planner_msgs::srv::ContainerOccupancyGridRegistration>(
    kContainerOccupancyGridRegistrationService,
    [this](const std::shared_ptr<cargo_planner_msgs::srv::ContainerOccupancyGridRegistration::Request> req,
           std::shared_ptr<cargo_planner_msgs::srv::ContainerOccupancyGridRegistration::Response> res) {
      onContainerOccupancyGridRegistration(req, res);
    });

  srv_cargo_list_registration_ = create_service<cargo_planner_msgs::srv::CargoListRegistration>(
    kCargoListRegistrationService,
    [this](const std::shared_ptr<cargo_planner_msgs::srv::CargoListRegistration::Request> req,
           std::shared_ptr<cargo_planner_msgs::srv::CargoListRegistration::Response> res) {
      onCargoListRegistration(req, res);
    });

  // ── Action server ─────────────────────────────────────────────────────────
  // cargo_planning is an action (not a service) because the placement algorithm is
  // CPU-intensive.  Running it as an action lets the caller receive a
  // "computing..." feedback immediately and poll for the result without
  // blocking other nodes or services.
  action_plan_cargo_ = rclcpp_action::create_server<PlanCargoAction>(
    this,
    kPlanCargoAction,
    [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PlanCargoAction::Goal> goal) {
      return onGoalReceived(uuid, goal);
    },
    [this](std::shared_ptr<GoalHandlePlanCargo> handle) {
      return onCancelReceived(handle);
    },
    [this](std::shared_ptr<GoalHandlePlanCargo> handle) {
      onGoalAccepted(handle);
    });

  // ── Publishers ───────────────────────────────────────────────────────────
  pub_markers_  = create_publisher<visualization_msgs::msg::MarkerArray>(kPlacementMarkersTopic,
                                                                         rclcpp::QoS(1).transient_local());
  pub_free_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>(kFreeMapTopic, rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(get_logger(),
              "cargo_planner ready. "
              "Services: %s, %s  Action: %s",
              kContainerOccupancyGridRegistrationService,
              kCargoListRegistrationService,
              kPlanCargoAction);
}

// ── Service: container_occupancy_grid_registration ────────────────────────────

void CargoPlacerNode::onContainerOccupancyGridRegistration(
  const std::shared_ptr<cargo_planner_msgs::srv::ContainerOccupancyGridRegistration::Request> req,
  std::shared_ptr<cargo_planner_msgs::srv::ContainerOccupancyGridRegistration::Response> res)
{
  if(req->grid_map.data.empty())
  {
    res->success = false;
    res->message = "Received empty OccupancyGrid.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  if(req->container_height <= 0.0)
  {
    res->success = false;
    res->message = "Received invalid container_height. It must be greater than 0.0 m.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  try
  {
    const cv::Mat free_mat = occupancyGridToMat(req->grid_map, occupied_threshold_);
    const double res_m     = req->grid_map.info.resolution;
    const double height    = req->container_height;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      planner_.setFreeSpace(free_mat, res_m, height);
      current_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(req->grid_map);
      has_map_      = true;
    }

    res->success = true;
    res->message = "Container occupancy grid stored.";
    RCLCPP_INFO(get_logger(),
                "Container occupancy grid stored: %ux%u cells @ %.3f m/cell (%.1f × %.1f m).",
                req->grid_map.info.width,
                req->grid_map.info.height,
                res_m,
                req->grid_map.info.width * res_m,
                req->grid_map.info.height * res_m);
  }
  catch(const std::exception& e)
  {
    res->success = false;
    res->message = std::string("Failed to process container occupancy grid: ") + e.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

// ── Service: cargo_list_registration ──────────────────────────────────────────

void CargoPlacerNode::onCargoListRegistration(
  const std::shared_ptr<cargo_planner_msgs::srv::CargoListRegistration::Request> req,
  std::shared_ptr<cargo_planner_msgs::srv::CargoListRegistration::Response> res)
{
  if(req->cargo_units.empty())
  {
    res->success = false;
    res->message = "Cargo list is empty.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  std::vector<CargoUnit> cargo_units;
  cargo_units.reserve(req->cargo_units.size());
  for(const auto& cargo_unit: req->cargo_units)
  {
    cargo_units.push_back(fromMsg(cargo_unit));
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    planner_.setCargoUnits(std::move(cargo_units));
    has_cargo_units_ = true;
    cargo_count_     = req->cargo_units.size();
  }

  res->success = true;
  res->message = "Cargo list stored (" + std::to_string(req->cargo_units.size()) + " cargo units).";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

// ── Action server: cargo_planning ─────────────────────────────────────────────

rclcpp_action::GoalResponse CargoPlacerNode::onGoalReceived(const rclcpp_action::GoalUUID& /*uuid*/,
                                                            std::shared_ptr<const PlanCargoAction::Goal> /*goal*/)
{
  if(planning_.exchange(true))
  {
    RCLCPP_WARN(get_logger(), "cargo_planning: goal rejected — another plan is already running.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CargoPlacerNode::onCancelReceived(std::shared_ptr<GoalHandlePlanCargo> /*handle*/)
{
  RCLCPP_INFO(get_logger(), "cargo_planning: cancel requested.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CargoPlacerNode::onGoalAccepted(std::shared_ptr<GoalHandlePlanCargo> handle)
{
  // Detach a thread so the ROS executor is not blocked while plan() runs.
  std::thread([this, handle]() {
    executePlan(handle);
  }).detach();
}

void CargoPlacerNode::executePlan(std::shared_ptr<GoalHandlePlanCargo> handle)
{
  // ── Guard: reset planning_ flag on return, whether via success or abort ──
  struct PlanningGuard
  {
    std::atomic<bool>& flag;
    ~PlanningGuard()
    {
      flag = false;
    }
  } guard{planning_};

  auto feedback = std::make_shared<PlanCargoAction::Feedback>();
  auto result   = std::make_shared<PlanCargoAction::Result>();

  // ── Check preconditions and publish initial feedback ─────────────────────
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if(!has_map_)
    {
      result->success = false;
      result->message = "No container grid set. Call container_occupancy_grid_registration first.";
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      handle->abort(result);
      return;
    }

    if(!has_cargo_units_)
    {
      result->success = false;
      result->message = "No cargo list set. Call cargo_list_registration first.";
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      handle->abort(result);
      return;
    }

    feedback->status       = "computing... (" + std::to_string(cargo_count_) + " cargo units)";
    feedback->cargo_total  = static_cast<uint32_t>(cargo_count_);
    feedback->cargo_placed = 0;
  }

  handle->publish_feedback(feedback);
  RCLCPP_INFO(get_logger(), "cargo_planning: %s", feedback->status.c_str());

  // ── Run the placement algorithm ──────────────────────────────────────────
  CargoPlanner::Result plan_result;
  double resolution = 0.05;
  double origin_x   = 0.0;
  double origin_y   = 0.0;
  std::string frame_id;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if(handle->is_canceling())
    {
      result->success = false;
      result->message = "Cancelled before computation started.";
      handle->canceled(result);
      return;
    }

    try
    {
      plan_result = planner_.plan();
      resolution  = current_grid_->info.resolution;
      frame_id    = current_grid_->header.frame_id;
      origin_x    = current_grid_->info.origin.position.x;
      origin_y    = current_grid_->info.origin.position.y;
    }
    catch(const std::exception& e)
    {
      result->success = false;
      result->message = std::string("plan() raised an exception: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      handle->abort(result);
      return;
    }
  }

  if(frame_id.empty())
  {
    RCLCPP_WARN(get_logger(),
                "OccupancyGrid has no frame_id. Cargo poses are valid (metres from grid origin) "
                "but RViz markers cannot be localised in the TF tree.");
  }

  // ── Fill result ─────────────────────────────────────────────────────────
  result->success = true;
  result->message = "Plan computed.";
  result->placements.reserve(plan_result.placements.size());
  for(const auto& p: plan_result.placements)
  {
    result->placements.push_back(toMsg(p, resolution, origin_x, origin_y));
  }
  result->no_placed       = plan_result.no_placed;
  result->placed_area_m2  = plan_result.placed_area_m2;
  result->free_area_m2    = plan_result.free_area_m2;
  result->total_area_m2   = plan_result.total_area_m2;
  result->utilization_pct = plan_result.utilization_pct;

  // ── Publish RViz visualisation ────────────────────────────────────────────
  const rclcpp::Time now = get_clock()->now();
  pub_markers_->publish(createDeleteAllMarkers(frame_id, now));
  pub_markers_->publish(createPlacementMarkers(plan_result.placements, resolution, frame_id, now));

  if(current_grid_)
  {
    auto updated         = matToOccupancyGrid(plan_result.final_work_map, *current_grid_);
    updated.header.stamp = now;
    pub_free_map_->publish(updated);
  }

  // ── Log summary ───────────────────────────────────────────────────────────
  RCLCPP_INFO(get_logger(),
              "cargo_planning: %zu placed, %zu no_placed, %.1f%% utilisation.",
              plan_result.placements.size(),
              plan_result.no_placed.size(),
              plan_result.utilization_pct);

  if(!plan_result.no_placed.empty())
  {
    std::ostringstream oss;
    for(const auto& id: plan_result.no_placed)
    {
      oss << " " << id;
    }
    RCLCPP_WARN(get_logger(), "Cargo units that did not fit:%s", oss.str().c_str());
  }

  handle->succeed(result);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

CargoUnit CargoPlacerNode::fromMsg(const cargo_planner_msgs::msg::CargoUnit& msg)
{
  CargoUnit cargo_unit;
  cargo_unit.id = msg.id;
  cargo_unit.lx = msg.lx;
  cargo_unit.ly = msg.ly;
  cargo_unit.lz = msg.lz;
  return cargo_unit;
}

cargo_planner_msgs::msg::CargoPlacement CargoPlacerNode::toMsg(const PlacementResult& r,
                                                               double resolution,
                                                               double origin_x,
                                                               double origin_y)
{
  cargo_planner_msgs::msg::CargoPlacement msg;
  msg.id      = r.id;
  msg.rotated = r.rotated;

  // Centre position in metres relative to the grid origin (cell 0,0).
  // For truck_inspector grids the origin is the bottom-right door corner
  // of the truck container, so these are coordinates in the truck frame.
  // origin_x/y account for a non-zero OccupancyGrid.info.origin.
  msg.pose.position.x = origin_x + r.anchor_col * resolution + r.effective_lx / 2.0;
  msg.pose.position.y = origin_y + r.anchor_row * resolution + r.effective_ly / 2.0;
  msg.pose.position.z = r.lz / 2.0;

  // Orientation as yaw quaternion around Z axis
  const double yaw       = r.rotated ? (M_PI / 2.0) : 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = std::sin(yaw / 2.0);
  msg.pose.orientation.w = std::cos(yaw / 2.0);

  return msg;
}

}  // namespace cargo_planner
