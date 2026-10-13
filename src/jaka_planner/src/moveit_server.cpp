#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "jaka_planner/JAKAZuRobot.h"
#include "jaka_planner/jkerr.h"
#include "jaka_planner/jktypes.h"

#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <string>
#include <map>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
using namespace std;

JAKAZuRobot robot;
const double PI = 3.1415926;
// bool in_pos;
// int ret_preempt;
// int ret_inPos;

typedef rclcpp_action::Server<control_msgs::action::FollowJointTrajectory> Server;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;
mutex robot_mutex;
atomic_bool execution_active{false};

// Map error codes to messages
map<int, string> mapErr = {
    {2,   "ERR_FUCTION_CALL_ERROR"},
    {-1,  "ERR_INVALID_HANDLER"},
    {-2,  "ERR_INVALID_PARAMETER"},
    {-3,  "ERR_COMMUNICATION_ERR"},
    {-4,  "ERR_KINE_INVERSE_ERR"},
    {-5,  "ERR_EMERGENCY_PRESSED"},
    {-6,  "ERR_NOT_POWERED"},
    {-7,  "ERR_NOT_ENABLED"},
    {-8,  "ERR_DISABLE_SERVOMODE"},
    {-9,  "ERR_NOT_OFF_ENABLE"},
    {-10, "ERR_PROGRAM_IS_RUNNING"},
    {-11, "ERR_CANNOT_OPEN_FILE"},
    {-12, "ERR_MOTION_ABNORMAL"},
    {-20, "ERR_PROTECTIVE_STOP"},
    {-21, "ERR_EMERGENCY_STOP"},
    {-22, "ERR_SOFT_LIMIT"},
    {-50, "ERR_MOTION_TIMEOUT"},
    {-51, "ERR_POWERON_TIMEOUT"},
    {-53, "ERR_ENABLE_TIMEOUT"},
    {-61, "ERR_OPERATION_TIMEOUT"}
};

string sdkError(int code)
{
    const auto known_error = mapErr.find(code);
    return known_error == mapErr.end() ? "SDK error " + to_string(code) : known_error->second;
}

void disableServoMode()
{
    lock_guard<mutex> lock(robot_mutex);
    robot.servo_move_enable(false);
}

bool robotFault(string &reason)
{
    RobotStatus_simple robot_status{};
    MotionStatus motion_status{};

    lock_guard<mutex> lock(robot_mutex);
    const int robot_status_result = robot.get_robot_status_simple(&robot_status);
    if (robot_status_result != 0)
    {
        reason = "Cannot read robot status: " + sdkError(robot_status_result);
        return true;
    }

    if (robot_status.errcode != 0)
    {
        ostringstream message;
        message << "Controller error " << robot_status.errcode;
        if (robot_status.errmsg[0] != '\0')
        {
            message << ": " << robot_status.errmsg;
        }
        reason = message.str();
        return true;
    }
    if (!robot_status.powered_on)
    {
        reason = "Robot power is off";
        return true;
    }
    if (!robot_status.enabled)
    {
        reason = "Robot is not enabled";
        return true;
    }

    const int motion_status_result = robot.get_motion_status(&motion_status);
    if (motion_status_result != 0)
    {
        reason = "Cannot read motion status: " + sdkError(motion_status_result);
        return true;
    }
    if (motion_status.isInEstop)
    {
        reason = "Emergency stop is active";
        return true;
    }
    if (motion_status.isInCollision)
    {
        reason = "Protective stop/collision state is active";
        return true;
    }
    if (motion_status.isOnLimit)
    {
        reason = "Robot reached a soft joint limit";
        return true;
    }
    return false;
}

bool controllerProgramPaused(string &error)
{
    ProgramState program_state = PROGRAM_IDLE;
    int sdk_result;
    {
        lock_guard<mutex> lock(robot_mutex);
        sdk_result = robot.get_program_state(&program_state);
    }
    if (sdk_result != 0)
    {
        error = "Cannot read controller program state: " + sdkError(sdk_result);
        return true;
    }
    if (program_state == PROGRAM_PAUSED)
    {
        error = "Controller program is paused. Run recover_robot.sh before SDK servo control";
        return true;
    }
    if (program_state == PROGRAM_RUNNING)
    {
        error = "Controller program is running. Stop it before SDK servo control";
        return true;
    }
    return false;
}

// Determine if the robot has reached the target within 0.2 degrees.
bool targetReached(const JointValue &joint_pose, double &max_error_deg, string &error)
{
    JointValue joint_position{};
    int sdk_result;
    {
        lock_guard<mutex> lock(robot_mutex);
        sdk_result = robot.get_joint_position(&joint_position);
    }
    if (sdk_result != 0)
    {
        error = "Cannot read joint position: " + sdkError(sdk_result);
        return false;
    }

    max_error_deg = 0.0;
    for (int i = 0; i < 6; i++)
    {
        const double error_deg = abs(joint_position.jVal[i] - joint_pose.jVal[i]) * 180.0 / PI;
        max_error_deg = max(max_error_deg, error_deg);
    }
    return max_error_deg <= 0.2;
}

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

void abortGoal(const shared_ptr<GoalHandle> &goal_handle, int error_code, const string &message)
{
    {
        lock_guard<mutex> lock(robot_mutex);
        robot.motion_abort();
        robot.servo_move_enable(false);
    }
    auto result = make_shared<FollowJointTrajectory::Result>();
    result->error_code = error_code;
    result->error_string = message;
    RCLCPP_ERROR(rclcpp::get_logger("goalCb"), "%s", message.c_str());
    goal_handle->abort(result);
}

void cancelGoal(const shared_ptr<GoalHandle> &goal_handle)
{
    {
        lock_guard<mutex> lock(robot_mutex);
        robot.motion_abort();
        robot.servo_move_enable(false);
    }
    auto result = make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    result->error_string = "Trajectory canceled";
    goal_handle->canceled(result);
    RCLCPP_WARN(rclcpp::get_logger("goalCb"), "Trajectory canceled");
}

// Handle a new FollowJointTrajectory goal
void goalCb(const shared_ptr<GoalHandle> goal_handle)
{
    struct ExecutionGuard
    {
        ~ExecutionGuard() { execution_active.store(false); }
    } execution_guard;

    string preflight_error;
    if (controllerProgramPaused(preflight_error))
    {
        auto result = make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = preflight_error;
        RCLCPP_ERROR(rclcpp::get_logger("goalCb"), "%s", preflight_error.c_str());
        goal_handle->abort(result);
        return;
    }

    // Retrieve the trajectory from the goal
    auto goal = goal_handle->get_goal();
    const auto &traj = goal->trajectory;
    int point_num = traj.points.size();
    RCLCPP_INFO(rclcpp::get_logger("goalCb"), "number of points: %d", point_num);

    if (point_num < 2)
    {
        abortGoal(goal_handle, FollowJointTrajectory::Result::INVALID_GOAL,
                  "Trajectory must contain at least two points");
        return;
    }

    for (const auto &point : traj.points)
    {
        if (point.positions.size() < 6)
        {
            abortGoal(goal_handle, FollowJointTrajectory::Result::INVALID_JOINTS,
                      "Trajectory point contains fewer than six joint positions");
            return;
        }
        for (int j = 0; j < 6; j++)
        {
            constexpr double max_absolute_angle = 6.283185307179586;
            const double angle = point.positions[j];
            if (!isfinite(angle) || abs(angle) > max_absolute_angle)
            {
                ostringstream message;
                message << "Invalid trajectory angle for joint " << j + 1
                        << ": " << angle
                        << " rad; allowed absolute range is [-360, 360] deg";
                auto result = make_shared<FollowJointTrajectory::Result>();
                result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
                result->error_string = message.str();
                RCLCPP_ERROR(rclcpp::get_logger("goalCb"), "%s", message.str().c_str());
                goal_handle->abort(result);
                return;
            }
        }
    }

    // Never enter servo mode when MoveIt's planned start differs from the
    // physical robot. Such a mismatch makes the first servo command a jump.
    JointValue actual_start{};
    int sdk_result;
    {
        lock_guard<mutex> lock(robot_mutex);
        sdk_result = robot.get_joint_position(&actual_start);
    }
    if (sdk_result != 0)
    {
        abortGoal(goal_handle, FollowJointTrajectory::Result::INVALID_GOAL,
                  "Cannot verify trajectory start: " + sdkError(sdk_result));
        return;
    }

    constexpr double max_start_error_deg = 2.0;
    double largest_start_error_deg = 0.0;
    int largest_start_error_joint = 0;
    for (int j = 0; j < 6; j++)
    {
        const double error_deg =
            abs(actual_start.jVal[j] - traj.points.front().positions[j]) * 180.0 / PI;
        if (error_deg > largest_start_error_deg)
        {
            largest_start_error_deg = error_deg;
            largest_start_error_joint = j + 1;
        }
    }
    if (largest_start_error_deg > max_start_error_deg)
    {
        ostringstream message;
        message << "Trajectory start does not match the physical robot: joint "
                << largest_start_error_joint << " differs by "
                << largest_start_error_deg << " deg (limit "
                << max_start_error_deg << " deg). Replan from the current state";
        abortGoal(goal_handle, FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP,
                  message.str());
        return;
    }

    {
        lock_guard<mutex> lock(robot_mutex);
        sdk_result = robot.servo_move_enable(true);
    }
    if (sdk_result != 0)
    {
        abortGoal(goal_handle, FollowJointTrajectory::Result::INVALID_GOAL,
                  "Cannot enable servo mode: " + sdkError(sdk_result));
        return;
    }

    float lastDuration = 0.0;
    JointValue joint_pose{};

    for (int i = 1; i < point_num; i++)
    {
        if (goal_handle->is_canceling())
        {
            cancelGoal(goal_handle);
            return;
        }

        // Grab the positions from the i-th trajectory point
        for (int j = 0; j < 6; j++) {
            joint_pose.jVal[j] = traj.points[i].positions[j];
        }

        // Convert time_from_start to a float seconds
        float Duration = static_cast<float>(traj.points[i].time_from_start.sec) +
                         static_cast<float>(traj.points[i].time_from_start.nanosec) * 1e-9;

        // Calculate time delta relative to previous point
        float dt = Duration - lastDuration;
        lastDuration = Duration;

        if (!isfinite(dt) || dt <= 0.0f)
        {
            abortGoal(goal_handle, FollowJointTrajectory::Result::INVALID_GOAL,
                      "Trajectory timestamps must be finite and strictly increasing");
            return;
        }

        // step_num matches old ROS1 logic: step_num = dt / 0.008
        int step_num = static_cast<int>(dt / 0.008f);
        step_num = max(step_num, 1);

        {
            lock_guard<mutex> lock(robot_mutex);
            sdk_result = robot.servo_j(&joint_pose, MoveMode::ABS, step_num);
        }
        if (sdk_result != 0)
        {
            string reason;
            robotFault(reason);
            if (reason.empty())
            {
                reason = sdkError(sdk_result);
            }
            abortGoal(goal_handle, FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
                      "servo_j failed at trajectory point " + to_string(i) + ": " + reason);
            return;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("goalCb"), "The return status of servo_j: %d", sdk_result);
        RCLCPP_INFO(rclcpp::get_logger("goalCb"), 
                    "For point no.: %d, Accepted joint angle: %f %f %f %f %f %f, dt=%f, step_num=%d", i,
                    joint_pose.jVal[0], joint_pose.jVal[1], joint_pose.jVal[2],
                    joint_pose.jVal[3], joint_pose.jVal[4], joint_pose.jVal[5],
                    dt, step_num);

    }

    // Keep an uninterrupted stream while the LPF settles. SDK reads between
    // servo_j calls create gaps at exactly the point where the robot decelerates.
    const auto settle_deadline = chrono::steady_clock::now() + chrono::seconds(2);
    while (rclcpp::ok() && chrono::steady_clock::now() < settle_deadline)
    {
        if (goal_handle->is_canceling())
        {
            cancelGoal(goal_handle);
            return;
        }

        {
            lock_guard<mutex> lock(robot_mutex);
            sdk_result = robot.servo_j(&joint_pose, MoveMode::ABS, 12);
        }
        if (sdk_result != 0)
        {
            string fault_reason;
            robotFault(fault_reason);
            if (fault_reason.empty())
            {
                fault_reason = sdkError(sdk_result);
            }
            abortGoal(goal_handle, FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED,
                      "Cannot hold final trajectory point: " + fault_reason);
            return;
        }
    }

    disableServoMode();

    string read_error;
    double max_error_deg = 0.0;
    if (targetReached(joint_pose, max_error_deg, read_error))
    {
        auto result = make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "Target reached";
        goal_handle->succeed(result);
        RCLCPP_INFO(rclcpp::get_logger("goalCb"),
                    "Target reached; max joint error %.3f deg", max_error_deg);
        return;
    }
    if (!read_error.empty())
    {
        abortGoal(goal_handle, FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
                  read_error);
        return;
    }

    string fault_reason;
    if (robotFault(fault_reason))
    {
        abortGoal(goal_handle, FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
                  "Robot stopped before reaching the target: " + fault_reason);
        return;
    }

    ostringstream tolerance_error;
    tolerance_error << "Robot did not reach the final trajectory point; max joint error "
                    << max_error_deg << " deg";
    abortGoal(goal_handle, FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED,
              tolerance_error.str());
}

// Publish the robot's joint states to /joint_states (for RViz / MoveIt feedback)
void joint_states_callback(rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr &joint_states_pub)
{
    // JAKA requires an uninterrupted servo_j stream. Reading joint positions
    // through the same SDK handle between servo commands can introduce gaps.
    if (execution_active.load())
    {
        return;
    }

    sensor_msgs::msg::JointState joint_msg;
    JointValue joint_position{};
    int sdk_result;
    {
        lock_guard<mutex> lock(robot_mutex);
        sdk_result = robot.get_joint_position(&joint_position);
    }
    if (sdk_result != 0)
    {
        static rclcpp::Clock throttle_clock;
        RCLCPP_ERROR_THROTTLE(
            rclcpp::get_logger("jointStates"), throttle_clock, 2000,
            "Failed to read joint positions: %s", sdkError(sdk_result).c_str());
        return;
    }

    for (int i = 0; i < 6; i++)
    {
        // joint_msg.position.push_back(robotstatus.joint_position[i]);
        joint_msg.position.push_back(joint_position.jVal[i]);
        joint_msg.name.push_back("joint_" + to_string(i+1));
    }
    joint_msg.header.stamp = rclcpp::Clock().now();
    joint_states_pub->publish(joint_msg);
}

void sigintHandler(int /*sig*/) {
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);
    auto node = rclcpp::Node::make_shared("moveit_server");

    // Read parameters
    string default_ip = "10.5.5.100";
    string default_model = "zu3";
    string robot_ip = node->declare_parameter("ip", default_ip);
    string robot_model = node->declare_parameter("model", default_model);
    bool auto_enable = node->declare_parameter("auto_enable", true);
    double servo_lpf_cutoff_hz = node->declare_parameter("servo_lpf_cutoff_hz", 0.5);
    bool configure_payload = node->declare_parameter("configure_payload", false);
    double payload_mass_kg = node->declare_parameter("payload_mass_kg", 0.0);
    double payload_centroid_x_mm = node->declare_parameter("payload_centroid_x_mm", 0.0);
    double payload_centroid_y_mm = node->declare_parameter("payload_centroid_y_mm", 0.0);
    double payload_centroid_z_mm = node->declare_parameter("payload_centroid_z_mm", 0.0);

    if (!isfinite(servo_lpf_cutoff_hz) || servo_lpf_cutoff_hz <= 0.0)
    {
        RCLCPP_FATAL(
            node->get_logger(),
            "servo_lpf_cutoff_hz must be a finite positive value, got %.3f",
            servo_lpf_cutoff_hz);
        rclcpp::shutdown();
        return 1;
    }
    if (!isfinite(payload_mass_kg) || payload_mass_kg < 0.0 ||
        !isfinite(payload_centroid_x_mm) ||
        !isfinite(payload_centroid_y_mm) ||
        !isfinite(payload_centroid_z_mm))
    {
        RCLCPP_FATAL(node->get_logger(), "Payload parameters must be finite and mass must be non-negative");
        rclcpp::shutdown();
        return 1;
    }

    // Connect to robot
    robot.login_in(robot_ip.c_str(), false);
    rclcpp::Rate rate(125);

    // Turn off servo at startup
    robot.servo_move_enable(false);
    rclcpp::sleep_for(chrono::milliseconds(500));

    const int filter_result = robot.servo_move_use_joint_LPF(servo_lpf_cutoff_hz);
    if (filter_result != 0)
    {
        RCLCPP_FATAL(node->get_logger(), "Cannot configure JAKA servo LPF: %s",
                     sdkError(filter_result).c_str());
        robot.login_out();
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "JAKA servo joint LPF cutoff: %.3f Hz",
                servo_lpf_cutoff_hz);

    if (configure_payload)
    {
        PayLoad requested_payload{};
        requested_payload.mass = payload_mass_kg;
        requested_payload.centroid.x = payload_centroid_x_mm;
        requested_payload.centroid.y = payload_centroid_y_mm;
        requested_payload.centroid.z = payload_centroid_z_mm;

        const int set_payload_result = robot.set_payload(&requested_payload);
        if (set_payload_result != 0)
        {
            RCLCPP_FATAL(node->get_logger(), "Cannot configure JAKA payload: %s",
                         sdkError(set_payload_result).c_str());
            robot.login_out();
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(
            node->get_logger(),
            "Configured JAKA payload: mass=%.3f kg, centroid=(%.1f, %.1f, %.1f) mm",
            requested_payload.mass, requested_payload.centroid.x,
            requested_payload.centroid.y, requested_payload.centroid.z);
    }

    int collision_level = 0;
    const int collision_level_result = robot.get_collision_level(&collision_level);
    if (collision_level_result == 0)
    {
        RCLCPP_INFO(node->get_logger(), "JAKA collision level: %d", collision_level);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Cannot read collision level: %s",
                    sdkError(collision_level_result).c_str());
    }

    PayLoad payload{};
    const int payload_result = robot.get_payload(&payload);
    if (payload_result == 0)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "JAKA payload: mass=%.3f kg, centroid=(%.1f, %.1f, %.1f) mm",
            payload.mass, payload.centroid.x, payload.centroid.y, payload.centroid.z);
        if (configure_payload &&
            (abs(payload.mass - payload_mass_kg) > 0.001 ||
             abs(payload.centroid.x - payload_centroid_x_mm) > 0.1 ||
             abs(payload.centroid.y - payload_centroid_y_mm) > 0.1 ||
             abs(payload.centroid.z - payload_centroid_z_mm) > 0.1))
        {
            RCLCPP_FATAL(node->get_logger(), "Controller payload verification failed; refusing motion");
            robot.login_out();
            rclcpp::shutdown();
            return 1;
        }
    }
    else
    {
        if (configure_payload)
        {
            RCLCPP_FATAL(node->get_logger(), "Cannot verify JAKA payload: %s",
                         sdkError(payload_result).c_str());
            robot.login_out();
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_WARN(node->get_logger(), "Cannot read payload: %s",
                    sdkError(payload_result).c_str());
    }

    if (auto_enable)
    {
        // Power on + enable
        robot.power_on();
        rclcpp::sleep_for(chrono::seconds(8));
        robot.enable_robot();
        rclcpp::sleep_for(chrono::seconds(4));
    }
    else
    {
        RCLCPP_WARN(
            node->get_logger(),
            "Automatic power-on and enable are disabled. Enable the robot manually only after stopping controller programs.");
    }

    // Publisher for /joint_states
    joint_states_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Create Action Server for FollowJointTrajectory
    auto moveit_server = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        node,
        "/jaka_" + robot_model + "_controller/follow_joint_trajectory",
        // Goal callback
        [](const rclcpp_action::GoalUUID &uuid,
           shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal) {
            RCLCPP_INFO(rclcpp::get_logger("moveit_server"), "Received goal request");
            (void)uuid; // Avoid unused parameter warning
            if (goal->trajectory.points.size() < 2)
            {
                RCLCPP_ERROR(rclcpp::get_logger("moveit_server"), "Rejecting trajectory with fewer than two points");
                return rclcpp_action::GoalResponse::REJECT;
            }
            bool expected = false;
            if (!execution_active.compare_exchange_strong(expected, true))
            {
                RCLCPP_WARN(rclcpp::get_logger("moveit_server"), "Rejecting goal because another trajectory is active");
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // Cancel callback
        [](const shared_ptr<GoalHandle> goal_handle) {
            (void)goal_handle;
            RCLCPP_WARN(rclcpp::get_logger("moveit_server"), "Received cancel request");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        // Execute callback
        [](const shared_ptr<GoalHandle> goal_handle) {
            RCLCPP_INFO(rclcpp::get_logger("moveit_server"), "Executing goal");
            thread{goalCb, goal_handle}.detach();
        }
    );

    RCLCPP_INFO(rclcpp::get_logger("moveit_server"), "==================Moveit Start==================");

    // Main spin loop
    while (rclcpp::ok())
    {
        // Publish joint states at ~125Hz
        joint_states_callback(joint_states_pub);
        rate.sleep();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
