#include "DucoCobotHead.h"
#include "DucoCobotWaist.h"
#include <exception>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace DucoAPI;

const string ROBOT_IP = "192.168.120.225";
const unsigned int WAIST_PORT = 7020;
const unsigned int HEAD_PORT = 7021;

int main(int argc, char* argv[])
{
    if (argc == 2 && string(argv[1]) == "--help") {
        cout << "Usage: duco_components [ip]\nRead waist and head state only.\n";
        return 0;
    }
    if (argc > 2) {
        cerr << "Usage: duco_components [ip]" << endl;
        return 1;
    }

    const string ip = argc > 1 ? argv[1] : ROBOT_IP;
    DucoCobotWaist waist(ip, WAIST_PORT);
    DucoCobotHead head(ip, HEAD_PORT);
    int exit_code = 0;

    // 先建立全部连接，再进行查询；全部业务结束时才统一关闭。
    if (waist.open() != 0) {
        cerr << "Cannot connect to waist" << endl;
        return 1;
    }
    if (head.open() != 0) {
        cerr << "Cannot connect to head" << endl;
        waist.close();
        return 1;
    }

    // 腰部与头部使用不同的 RPC 服务，不能混用客户端。
    try {
        vector<int8_t> state;
        vector<double> joints;
        waist.get_robot_state(state);
        cout << "Waist state:";
        for (int8_t value : state) cout << " " << static_cast<int>(value);
        cout << endl;
        waist.get_actual_joints_position(joints);
        cout << "Waist joints (rad):";
        for (double value : joints) cout << " " << value;
        cout << endl;
    } catch (const exception& error) {
        cerr << "Waist query failed: " << error.what() << endl;
        exit_code = 1;
    }

    try {
        vector<int8_t> state;
        vector<double> joints;
        head.get_robot_state(state);
        cout << "Head state:";
        for (int8_t value : state) cout << " " << static_cast<int>(value);
        cout << endl;
        head.get_actual_joints_position(joints);
        cout << "Head joints (rad):";
        for (double value : joints) cout << " " << value;
        cout << endl;
    } catch (const exception& error) {
        cerr << "Head query failed: " << error.what() << endl;
        exit_code = 1;
    }

    // 此示例只查询。使能接口为 enable(axis_num, block)，轴号由控制器定义。
    // 运动前需像 Move.cpp 一样使用独立连接维护对应部件的心跳。
    // 后续查询继续复用 waist / head，结束全部业务后再关闭。
    head.close();
    waist.close();
    return exit_code;
}
