#pragma once

/**
 * @file cargo_planner_node.hpp
 * @brief ROS 2 node that wraps CargoPlanner and exposes its interfaces
 *        relative to the node namespace.
 *
 * Node lifecycle
 * --------------
 * The node starts immediately and is ready to accept interface calls. It
 * waits for two pieces of information before a plan can be computed:
 *
 *   1. A container grid: provided by calling container_occupancy_grid_registration
 *      directly or by using container_occupancy_grid_forwarder.py.
 *   2. A cargo list: provided by calling cargo_list_registration.
 *
 * Either piece can arrive first, in any order. cargo_planning will report an
 * error if called before both are available.
 *
 * Services
 * --------
 *   container_occupancy_grid_registration  (cargo_planner_msgs/ContainerOccupancyGridRegistration)
 *     Stores the OccupancyGrid of the container interior. Call after every
 *     LiDAR rescan. The node converts the grid to an internal cv::Mat and
 *     stores it for the next cargo_planning invocation.
 *
 *   cargo_list_registration  (cargo_planner_msgs/CargoListRegistration)
 *     Stores the list of cargo units to be loaded. The node sorts them by
 *     volume descending (largest first) before forwarding to CargoPlanner.
 *     Replaces any previous list (including YAML-loaded defaults).
 *
 * Action server
 * -------------
 *   cargo_planning  (cargo_planner_msgs/action/PlanCargo)
 *     Triggers the placement algorithm. The goal is empty; the server reads
 *     the map and cargo list already stored. The server:
 *       1. Accepts the goal immediately.
 *       2. Publishes a feedback message: "computing... (N cargo units)".
 *       3. Runs the CPU-intensive plan() in a detached thread so the ROS executor remains free.
 *       4. Reports the final placed count and returns the complete result.
 *     Concurrent cargo_planning goals are rejected.
 *
 * Topics published
 * ----------------
 *   cargo_markers  (visualization_msgs/MarkerArray)
 *     RViz cuboid markers for each placed cargo unit. Published after every
 *     successful cargo_planning action.
 *
 *   container_free_map (nav_msgs/OccupancyGrid)
 *     The truck's free-space grid after all cargo units have been placed.
 *     Useful for debugging and for chaining with other planners.
 *
 * Parameters (see config/default_cargo_planner.yaml)
 * ------------------------------------------
 *   occupied_threshold   Cells >= this value are treated as occupied [0..100].
 *   pallet_margin        Non-negative finite safety clearance around obstacles [m].
 *   enable_rotation      Whether the planner may place cargo at 90 degrees yaw.
 */

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cargo_planner_msgs/action/plan_cargo.hpp>
#include <cargo_planner_msgs/srv/cargo_list_registration.hpp>
#include <cargo_planner_msgs/srv/container_occupancy_grid_registration.hpp>

#include "cargo_planner/cargo_planner.hpp"
#include "cargo_planner/types.hpp"

namespace cargo_planner
{

  class CargoPlacerNode: public rclcpp::Node
  {
    public:
      explicit CargoPlacerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

    private:
      // Type aliases

      using PlanCargoAction = cargo_planner_msgs::action::PlanCargo;
      using GoalHandlePlanCargo = rclcpp_action::ServerGoalHandle<PlanCargoAction>;

      // Service handlers

      void onContainerOccupancyGridRegistration(
        const std::shared_ptr<cargo_planner_msgs::srv::ContainerOccupancyGridRegistration::Request> req,
        std::shared_ptr<cargo_planner_msgs::srv::ContainerOccupancyGridRegistration::Response> res);

      void onCargoListRegistration(const std::shared_ptr<cargo_planner_msgs::srv::CargoListRegistration::Request> req,
                                   std::shared_ptr<cargo_planner_msgs::srv::CargoListRegistration::Response> res);

      // Action server callbacks

      /**
       * @brief Called when a new cargo_planning goal is received.
       *
       * Always accepts unless another plan is already running.
       */
      rclcpp_action::GoalResponse onGoalReceived(const rclcpp_action::GoalUUID& uuid,
                                                 std::shared_ptr<const PlanCargoAction::Goal> goal);

      /** @brief Called when the action client requests a cancel.  Always accepted. */
      rclcpp_action::CancelResponse onCancelReceived(std::shared_ptr<GoalHandlePlanCargo> handle);

      /**
       * @brief Detach a planning thread while retaining shared ownership of the node.
       */
      void onGoalAccepted(std::shared_ptr<GoalHandlePlanCargo> handle);

      /**
       * @brief Entry point of the planning thread.
       *
       * Publishes feedback, runs plan(), fills the result, publishes RViz
       * markers, and calls handle->succeed() or handle->abort().
       */
      void executePlan(std::shared_ptr<GoalHandlePlanCargo> handle);

      // Helpers

      /** @brief Convert ROS CargoUnit msg to internal type. */
      static CargoUnit fromMsg(const cargo_planner_msgs::msg::CargoUnit& msg);

      /**
       * @brief Convert internal PlacementResult to ROS CargoPlacement msg.
       *
       * Positions are expressed in metres relative to the grid origin
       * (origin_x, origin_y), which corresponds to cell (0,0) of the
       * OccupancyGrid.  For truck_inspector grids this is the bottom-right
       * door corner of the truck container.
       *
       * @param r         Placement result from the planner.
       * @param resolution Cell size [m].
       * @param origin_x  X coordinate of cell (0,0) in the grid frame [m].
       * @param origin_y  Y coordinate of cell (0,0) in the grid frame [m].
       */
      static cargo_planner_msgs::msg::CargoPlacement toMsg(const PlacementResult& r,
                                                           double resolution,
                                                           double origin_x,
                                                           double origin_y);

      // ROS interfaces

      rclcpp::Service<cargo_planner_msgs::srv::ContainerOccupancyGridRegistration>::SharedPtr
        srv_container_occupancy_grid_registration_;
      rclcpp::Service<cargo_planner_msgs::srv::CargoListRegistration>::SharedPtr srv_cargo_list_registration_;

      rclcpp_action::Server<PlanCargoAction>::SharedPtr action_plan_cargo_;

      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_free_map_;

      // Shared state (protected by state_mutex_)

      mutable std::mutex state_mutex_;
      CargoPlanner planner_;
      nav_msgs::msg::OccupancyGrid::SharedPtr current_grid_;  ///< Last received truck grid.
      bool has_map_{false};
      bool has_cargo_units_{false};
      std::size_t cargo_count_{0};  ///< Number of cargo units in the current list.

      /// Guards against concurrent cargo_planning goals.
      std::atomic<bool> planning_{false};

      // Parameters

      int occupied_threshold_;
      double pallet_margin_;
      bool enable_rotation_;
  };

}  // namespace cargo_planner
