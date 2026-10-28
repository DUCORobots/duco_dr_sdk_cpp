#include "DucoCobot.h"
#include <atomic>
#include <chrono>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std;
using namespace DucoAPI;

// 直接修改这里的配置，端口和是否运动不通过启动参数输入。
const string ROBOT_IP = "192.168.120.225";
const unsigned int LEFT_ARM_PORT = 7003;
const unsigned int RIGHT_ARM_PORT = 7004;
const bool RUN_MOTION = false; // 改为 true 后，左臂运动结束再执行右臂运动。

int main(int argc, char* argv[])
{
    if (argc == 2 && string(argv[1]) == "--help") {
        cout << "Usage: duco_move [ip]\n"
             << "Run the left arm first, then the right arm.\n";
        return 0;
    }
    if (argc > 2) {
        cerr << "Usage: duco_move [ip]" << endl;
        return 1;
    }
    const string ip = argc > 1 ? argv[1] : ROBOT_IP;

    DucoCobot left_arm(ip, LEFT_ARM_PORT);
    DucoCobot right_arm(ip, RIGHT_ARM_PORT);
    // 心跳线程分别使用独立对象，四个连接由 main 持有直到全部业务结束。
    DucoCobot left_heartbeat(ip, LEFT_ARM_PORT);
    DucoCobot right_heartbeat(ip, RIGHT_ARM_PORT);
    atomic<bool> stop_heartbeat(false);
    atomic<bool> heartbeat_failed(false);
    thread left_heartbeat_thread, right_heartbeat_thread;
    bool left_open = false, right_open = false;
    bool left_heartbeat_open = false, right_heartbeat_open = false;
    DucoCobot* moving_arm = nullptr;
    int exit_code = 0;

    try {
        if (left_arm.open() != 0) throw runtime_error("Cannot connect to left arm");
        left_open = true;
        if (right_arm.open() != 0) throw runtime_error("Cannot connect to right arm");
        right_open = true;

        if (RUN_MOTION) {
            if (left_heartbeat.open() != 0) throw runtime_error("Left heartbeat connection failed");
            left_heartbeat_open = true;
            if (right_heartbeat.open() != 0) throw runtime_error("Right heartbeat connection failed");
            right_heartbeat_open = true;

            // 先确认两侧心跳成功，再启动线程与运动。
            left_heartbeat.rpc_heartbeat(1000);
            right_heartbeat.rpc_heartbeat(1000);
            left_heartbeat_thread = thread([&] {
                try {
                    while (!stop_heartbeat.load()) {
                        this_thread::sleep_for(chrono::milliseconds(500));
                        if (stop_heartbeat.load()) break;
                        left_heartbeat.rpc_heartbeat(1000);
                    }
                } catch (const exception& error) {
                    cerr << "Left heartbeat failed: " << error.what() << endl;
                    heartbeat_failed.store(true);
                }
            });
            right_heartbeat_thread = thread([&] {
                try {
                    while (!stop_heartbeat.load()) {
                        this_thread::sleep_for(chrono::milliseconds(500));
                        if (stop_heartbeat.load()) break;
                        right_heartbeat.rpc_heartbeat(1000);
                    }
                } catch (const exception& error) {
                    cerr << "Right heartbeat failed: " << error.what() << endl;
                    heartbeat_failed.store(true);
                }
            });
        }

        // 只有这一处顺序循环：左臂整个流程结束后，才进入右臂流程。
        DucoCobot* arms[] = {&left_arm, &right_arm};
        const char* names[] = {"Left arm", "Right arm"};
        for (int arm_index = 0; arm_index < 2; ++arm_index) {
            DucoCobot& robot = *arms[arm_index];
            cout << "\n=== " << names[arm_index] << " ===" << endl;
            if (heartbeat_failed.load()) throw runtime_error("Heartbeat failed");

            // 1. 版本、状态与关节位置。
            string version;
            robot.get_robot_version(version);
            cout << "SDK: " << robot.get_version() << ", controller: " << version << endl;
            vector<int8_t> state;
            robot.get_robot_state(state);
            cout << "Robot/program/safety/mode:";
            for (int8_t value : state) cout << " " << static_cast<int>(value);
            cout << endl;

            vector<double> joints, pose;
            robot.get_actual_joints_position(joints);
            if (joints.empty()) throw runtime_error("No joint positions received");
            cout << "Actual joints (rad):";
            for (double value : joints) cout << " " << value;
            cout << endl;
            robot.get_tcp_pose(pose);
            cout << "TCP in base (m, rad):";
            for (double value : pose) cout << " " << value;
            cout << endl;

            // 2. 正逆解：空向量使用当前工具与工件，不执行逆解结果。
            vector<double> fk_pose, ik_joints;
            robot.cal_fkine(fk_pose, joints, {}, {});
            if (fk_pose.size() != 6) throw runtime_error("Invalid FK result");
            robot.cal_ikine(ik_joints, fk_pose, joints, {}, {});
            if (ik_joints.size() != joints.size()) throw runtime_error("Invalid IK result");
            cout << "FK in current work object (m, rad):";
            for (double value : fk_pose) cout << " " << value;
            cout << endl;
            cout << "IK joints (rad):";
            for (double value : ik_joints) cout << " " << value;
            cout << endl;

            // 3. 只读取数字 I/O，不改变输出。
            for (int16_t channel = 1; channel <= 2; ++channel) {
                cout << "DI" << channel << "=" << robot.get_tool_digital_in(channel)
                     << " DO" << channel << "=" << robot.get_tool_digital_out(channel) << endl;
            }

            // 4. RUN_MOTION=true 时，当前侧执行一次阻塞运动。
            if (RUN_MOTION) {
                // 设备应已上电；需要时按现场流程显式调用 robot.power_on(true)。
                int32_t result = robot.enable(true);
                cout << "enable: " << result << endl;
                if (result != ST_Finished) throw runtime_error("Enable did not finish");
                if (heartbeat_failed.load()) throw runtime_error("Heartbeat failed");

                vector<double> target;
                robot.get_actual_joints_position(target);
                if (target.empty()) throw runtime_error("No joint positions received");
                target[0] += 0.02; // 当前侧第一轴增加 0.02 rad，先确认限位与空间。
                if (heartbeat_failed.load()) throw runtime_error("Heartbeat failed");
                moving_arm = &robot; // 下发前标记，避免回复丢失时遗漏停止处理。
                result = robot.movej2(target, 0.1, 0.2, 0, true);
                cout << "movej2: " << result << endl;
                if (result != ST_Finished) throw runtime_error("Motion did not finish");
                moving_arm = nullptr;
                if (heartbeat_failed.load()) throw runtime_error("Heartbeat failed");
            }
            cout << names[arm_index] << " finished" << endl;
        }
    } catch (const exception& error) {
        cerr << "Error: " << error.what() << endl;
        exit_code = 1;
        if (moving_arm != nullptr) {
            try { cout << "stop: " << moving_arm->stop(true) << endl; }
            catch (const exception& stop_error) { cerr << "Stop failed: " << stop_error.what() << endl; }
        }
    }

    // 左臂流程结束时不关闭连接；全部业务结束或异常退出才统一清理。
    stop_heartbeat.store(true);
    if (left_heartbeat_thread.joinable()) left_heartbeat_thread.join();
    if (right_heartbeat_thread.joinable()) right_heartbeat_thread.join();
    if (right_heartbeat_open) right_heartbeat.close();
    if (left_heartbeat_open) left_heartbeat.close();
    if (right_open) right_arm.close();
    if (left_open) left_arm.close();
    return exit_code;
}
