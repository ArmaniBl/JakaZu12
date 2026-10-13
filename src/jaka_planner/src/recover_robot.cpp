#include "jaka_planner/JAKAZuRobot.h"
#include "jaka_planner/jktypes.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

namespace
{
const char *programStateName(ProgramState state)
{
    switch (state)
    {
        case PROGRAM_IDLE:
            return "IDLE";
        case PROGRAM_RUNNING:
            return "RUNNING";
        case PROGRAM_PAUSED:
            return "PAUSED";
    }
    return "UNKNOWN";
}

bool report(const char *operation, int result, bool required = true)
{
    std::cout << operation << ": " << result << (result == 0 ? " (OK)" : " (FAILED)") << '\n';
    return result == 0 || !required;
}
}  // namespace

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: recover_robot <robot-ip>\n";
        return 2;
    }

    JAKAZuRobot robot;
    if (!report("login_in", robot.login_in(argv[1], false)))
    {
        return 1;
    }

    ProgramState initial_program_state = PROGRAM_IDLE;
    const int initial_program_result = robot.get_program_state(&initial_program_state);
    if (initial_program_result == 0)
    {
        std::cout << "Initial program state: " << programStateName(initial_program_state) << '\n';
    }
    else
    {
        report("get_program_state (initial)", initial_program_result, false);
    }

    // program_abort changes RUNNING or PAUSED into IDLE. It does not start motion.
    bool ok = report("program_abort", robot.program_abort());
    ok = report("motion_abort", robot.motion_abort()) && ok;
    ok = report("servo_move_enable(false)", robot.servo_move_enable(false), false) && ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    BOOL in_collision = FALSE;
    const int collision_state_result = robot.is_in_collision(&in_collision);
    if (!report("is_in_collision", collision_state_result))
    {
        ok = false;
    }
    else if (in_collision)
    {
        ok = report("collision_recover", robot.collision_recover()) && ok;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    ok = report("clear_error", robot.clear_error()) && ok;

    RobotStatus_simple status{};
    int status_result = robot.get_robot_status_simple(&status);
    if (!report("get_robot_status_simple", status_result))
    {
        ok = false;
    }
    else
    {
        std::cout << "Before enable: powered_on=" << status.powered_on
                  << ", enabled=" << status.enabled
                  << ", error=" << status.errcode << '\n';

        if (!status.powered_on)
        {
            ok = report("power_on", robot.power_on()) && ok;
            std::this_thread::sleep_for(std::chrono::seconds(8));
        }
        if (!status.enabled)
        {
            ok = report("enable_robot", robot.enable_robot()) && ok;
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }

    // Verify actual SDK readiness. Entering servo mode does not move the robot;
    // no servo_j target is sent by this utility.
    const int final_clear_result = robot.clear_error();
    ok = report("clear_error (final)", final_clear_result) && ok;

    const int servo_probe_result = robot.servo_move_enable(true);
    ok = report("servo_move_enable(true) readiness check", servo_probe_result) && ok;
    BOOL in_servo_mode = FALSE;
    if (servo_probe_result == 0)
    {
        const int in_servo_result = robot.is_in_servomove(&in_servo_mode);
        ok = report("is_in_servomove", in_servo_result) && ok;
        ok = in_servo_mode && ok;
        ok = report("servo_move_enable(false) after check", robot.servo_move_enable(false)) && ok;
    }

    ProgramState final_program_state = PROGRAM_PAUSED;
    const int final_program_result = robot.get_program_state(&final_program_state);
    ok = report("get_program_state (final)", final_program_result) && ok;

    RobotStatus_simple final_status{};
    const int final_status_result = robot.get_robot_status_simple(&final_status);
    ok = report("get_robot_status_simple (final)", final_status_result) && ok;

    BOOL final_collision = FALSE;
    const int final_collision_result = robot.is_in_collision(&final_collision);
    ok = report("is_in_collision (final)", final_collision_result) && ok;

    if (final_program_result == 0)
    {
        std::cout << "Final program state: " << programStateName(final_program_state) << '\n';
        ok = (final_program_state == PROGRAM_IDLE) && ok;
    }
    if (final_status_result == 0)
    {
        std::cout << "Final robot state: powered_on=" << final_status.powered_on
                  << ", enabled=" << final_status.enabled
                  << ", error=" << final_status.errcode;
        if (final_status.errmsg[0] != '\0')
        {
            std::cout << " (" << final_status.errmsg << ")";
        }
        std::cout << '\n';
        if (final_status.errcode != 0 && servo_probe_result == 0)
        {
            std::cout << "NOTE: the controller still reports its previous error, but the "
                         "servo readiness check succeeded.\n";
        }
        ok = final_status.powered_on && final_status.enabled && ok;
    }
    if (final_collision_result == 0)
    {
        std::cout << "Final collision state: " << final_collision << '\n';
        ok = !final_collision && ok;
    }

    report("login_out", robot.login_out(), false);
    if (!ok)
    {
        std::cerr << "Recovery did not reach a ready state. Do not start MoveIt motion.\n";
        return 1;
    }

    std::cout << "Robot is powered, enabled, collision-free, and the controller program is IDLE.\n";
    return 0;
}
