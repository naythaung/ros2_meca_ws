#include <exception>
#include <memory>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char* argv[])
{
    // 1. Initialise ROS and create our node.
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>(
        "meca_move_to_pose",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    auto const logger = node->get_logger();

    // Process incoming ROS messages while we wait for MoveIt.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    int exit_code = 1;

    try {
        // 2. Connect to our existing MoveIt planning group.
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(node, "meca_arm");

        move_group_interface.setPoseReferenceFrame("world");
        move_group_interface.setEndEffectorLink("link_6");

        move_group_interface.setPlanningPipelineId("ompl");
        move_group_interface.setPlannerId("RRTConnectkConfigDefault");
        move_group_interface.setPlanningTime(5.0);

        move_group_interface.setMaxVelocityScalingFactor(0.2);
        move_group_interface.setMaxAccelerationScalingFactor(0.2);

        // Our captured target was rounded, so allow small tolerances.
        move_group_interface.setGoalPositionTolerance(0.001);
        move_group_interface.setGoalOrientationTolerance(0.01);

        // 3. Set the target pose, using the tutorial's lambda format.
        auto const target_pose = [] {
            geometry_msgs::msg::Pose pose;

            pose.position.x = 0.120;
            pose.position.y = -0.130;
            pose.position.z = 0.194;

            pose.orientation.x = 0.0;
            pose.orientation.y = -0.7071067811865476;
            pose.orientation.z = 0.0;
            pose.orientation.w = -0.7071067811865476;

            return pose;
        }();

        if (!move_group_interface.getCurrentState(10.0)) {
            RCLCPP_ERROR(logger, "No current robot state received.");
        } else {
            move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPoseTarget(target_pose);

            // 4. Ask MoveIt for a plan.
            auto const [success, plan] = [&move_group_interface] {
                MoveGroupInterface::Plan result;
                auto const success = static_cast<bool>(
                    move_group_interface.plan(result));

                return std::make_pair(success, result);
            }();

            // 5. Inspect the result. Execution is not enabled yet.
            if (success) {
                RCLCPP_INFO(logger, "Plan found. Executing...");

                const auto result = move_group_interface.execute(plan);

                if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(logger, "Target reached successfully.");
        
                    exit_code = 0;
                } else {
                    RCLCPP_ERROR(logger, "Execution failed.");
                }
            } else {
                RCLCPP_ERROR(logger, "Planning failed. Nothing executed.");
            }
        }
    } catch (const std::exception& error) {
        RCLCPP_ERROR(logger, "Task failed: %s", error.what());
    }

    // 6. Stop message processing and shut down.
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return exit_code;
}