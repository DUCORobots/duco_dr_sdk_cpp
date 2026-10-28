#include "DucoCobotWaist.h"
#include "thrift/gen-cpp/RPCWaist.h"
#include "thrift/protocol/TBinaryProtocol.h"
#include "thrift/transport/TBufferTransports.h"
#include "thrift/transport/TSocket.h"
#include <exception>

namespace DucoAPI {

DucoCobotWaist::DucoCobotWaist(std::string ip, unsigned int port)
{
    using namespace apache::thrift::transport;
    _socket = std::make_shared<TSocket>(ip, port);
    _socket->setConnTimeout(3000);
    _socket->setKeepAlive(true);
    if (port > 9000)
        _transport = std::make_shared<TFramedTransport>(_socket);
    else
        _transport = std::make_shared<TBufferedTransport>(_socket);
    _protocol = std::make_shared<apache::thrift::protocol::TBinaryProtocol>(_transport);
    _client = std::make_shared<RPCWaistClient>(_protocol);
}

DucoCobotWaist::~DucoCobotWaist()
{
    close();
}

int32_t DucoCobotWaist::open()
{
    try { _transport->open(); }
    catch (const std::exception&) { return -1; }
    return 0;
}

int32_t DucoCobotWaist::close()
{
    try { _transport->close(); }
    catch (const std::exception&) { return -1; }
    return 0;
}

int32_t DucoCobotWaist::enable(int32_t axis_num, bool block)
{
    return _client->enable(axis_num, block);
}

int32_t DucoCobotWaist::disable(int32_t axis_num, bool block)
{
    return _client->disable(axis_num, block);
}

int32_t DucoCobotWaist::stop(bool block)
{
    return _client->stop(block);
}

int32_t DucoCobotWaist::pause(bool block)
{
    return _client->pause(block);
}

int32_t DucoCobotWaist::resume(bool block)
{
    return _client->resume(block);
}

void DucoCobotWaist::get_robot_state(std::vector<int8_t>& state)
{
    _client->get_robot_state(state);
}

void DucoCobotWaist::get_robot_status(RobotStatus& status)
{
    ::RobotStatus rpc_status;
    _client->get_robot_status(rpc_status);
    // 显式转换为 SDK 公共结构，保留全部字段。
    status.actual_joint_pos = rpc_status.actual_joint_pos;
    status.actual_joint_vel = rpc_status.actual_joint_vel;
    status.actual_joint_acc = rpc_status.actual_joint_acc;
    status.actual_joint_torq = rpc_status.actual_joint_torq;
    status.actual_joint_temp = rpc_status.actual_joint_temp;
    status.actual_joint_current = rpc_status.actual_joint_current;
    status.auxiliary_joint_vel = rpc_status.auxiliary_joint_vel;
    status.auxiliary_joint_acc = rpc_status.auxiliary_joint_acc;
    status.auxiliary_joint_torq = rpc_status.auxiliary_joint_torq;
    status.target_joint_pos = rpc_status.target_joint_pos;
    status.target_joint_vel = rpc_status.target_joint_vel;
    status.target_joint_acc = rpc_status.target_joint_acc;
    status.target_joint_torq = rpc_status.target_joint_torq;
    status.servo_errid = rpc_status.servo_errid;
    status.servo_statusword = rpc_status.servo_statusword;
    status.actual_tcp_pose = rpc_status.actual_tcp_pose;
    status.actual_tcp_vel = rpc_status.actual_tcp_vel;
    status.actual_tcp_acc = rpc_status.actual_tcp_acc;
    status.target_tcp_pose = rpc_status.target_tcp_pose;
    status.target_tcp_vel = rpc_status.target_tcp_vel;
    status.target_tcp_acc = rpc_status.target_tcp_acc;
}

void DucoCobotWaist::get_actual_joints_position(std::vector<double>& values)
{
    _client->get_actual_joints_position(values);
}

void DucoCobotWaist::get_target_joints_position(std::vector<double>& values)
{
    _client->get_target_joints_position(values);
}

void DucoCobotWaist::get_actual_joints_speed(std::vector<double>& values)
{
    _client->get_actual_joints_speed(values);
}

void DucoCobotWaist::get_target_joints_speed(std::vector<double>& values)
{
    _client->get_target_joints_speed(values);
}

void DucoCobotWaist::get_actual_joints_acceleration(std::vector<double>& values)
{
    _client->get_actual_joints_acceleration(values);
}

void DucoCobotWaist::get_target_joints_acceleration(std::vector<double>& values)
{
    _client->get_target_joints_acceleration(values);
}

int32_t DucoCobotWaist::movej(const std::vector<double>& joints_list, double v, double a, double r, bool block, bool def_acc)
{
    return _client->movej(joints_list, v, a, r, block, def_acc);
}

int32_t DucoCobotWaist::movej2(const std::vector<double>& joints_list, double v, double a, double r, bool block, bool def_acc)
{
    return _client->movej2(joints_list, v, a, r, block, def_acc);
}

int32_t DucoCobotWaist::servoj(const std::vector<double>& joints_list, double v, double a, bool block, double kp, double kd, double smooth_vel, double smooth_acc)
{
    return _client->servoj(joints_list, v, a, block, kp, kd, smooth_vel, smooth_acc);
}

int32_t DucoCobotWaist::get_noneblock_taskstate(int32_t id)
{
    return _client->get_noneblock_taskstate(id);
}

void DucoCobotWaist::rpc_heartbeat(int32_t time)
{
    _client->rpc_heartbeat(time);
}

void DucoCobotWaist::get_flange_pose(std::vector<double>& values)
{
    _client->get_flange_pose(values);
}

void DucoCobotWaist::get_flange_speed(std::vector<double>& values)
{
    _client->get_flange_speed(values);
}

void DucoCobotWaist::get_flange_acceleration(std::vector<double>& values)
{
    _client->get_flange_acceleration(values);
}

void DucoCobotWaist::cal_fkine(std::vector<double>& pose, const std::vector<double>& joints_position, const std::vector<double>& tool, const std::vector<double>& wobj)
{
    _client->cal_fkine(pose, joints_position, tool, wobj);
}

void DucoCobotWaist::cal_ikine(std::vector<double>& joints, const std::vector<double>& p, const std::vector<double>& q_near, const std::vector<double>& tool, const std::vector<double>& wobj)
{
    _client->cal_ikine(joints, p, q_near, tool, wobj);
}

int32_t DucoCobotWaist::movej_pose(const std::vector<double>& p, double v, double a, double r, const std::vector<double>& q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc)
{
    return _client->movej_pose(p, v, a, r, q_near, tool, wobj, block, def_acc);
}

int32_t DucoCobotWaist::movej_pose2(const std::vector<double>& p, double v, double a, double r, const std::vector<double>& q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc)
{
    return _client->movej_pose2(p, v, a, r, q_near, tool, wobj, block, def_acc);
}

int32_t DucoCobotWaist::movel(const std::vector<double>& p, double v, double a, double r, const std::vector<double>& q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc)
{
    return _client->movel(p, v, a, r, q_near, tool, wobj, block, def_acc);
}

int32_t DucoCobotWaist::tcp_move(const std::vector<double>& pose_offset, double v, double a, double r, const std::string& tool, bool block, bool def_acc)
{
    return _client->tcp_move(pose_offset, v, a, r, tool, block, def_acc);
}

int32_t DucoCobotWaist::servol(const std::vector<double>& pose_list, double v, double a, const std::vector<double>& q_near, const std::string& tool, const std::string& wobj, bool block, double kp, double kd, double smooth_vel, double smooth_acc)
{
    return _client->servol(pose_list, v, a, q_near, tool, wobj, block, kp, kd, smooth_vel, smooth_acc);
}

int32_t DucoCobotWaist::servoj_pose(const std::vector<double>& pose_list, double v, double a, const std::vector<double>& q_near, const std::string& tool, const std::string& wobj, bool block, double kp, double kd, double smooth_vel, double smooth_acc)
{
    return _client->servoj_pose(pose_list, v, a, q_near, tool, wobj, block, kp, kd, smooth_vel, smooth_acc);
}

} // namespace DucoAPI
