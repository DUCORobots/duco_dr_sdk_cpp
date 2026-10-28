#include "DucoCobot.h"
#include <iostream>
#include <map>
#ifdef WIN32
#define    WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <WinSock2.h>
#include <stdio.h>
#else
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <unistd.h>
#include <string>
#endif

#include <thrift/transport/TSocket.h>
#include <thrift/transport/TBufferTransports.h>
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/transport/TTransportException.h>
#include "thrift/gen-cpp/RPCRobot.h"

#ifdef WIN32
#pragma comment(lib,"ws2_32.lib")
#endif

#define PTHREAD_STACK_MIN	           16384
#define APP_STACKSIZE                  4096        /* bytes */
#define APP_PRIORITY                   85
#ifdef __cplusplus
#define CC_STATIC_ASSERT(exp) static_assert (exp, "")
#else
#define CC_STATIC_ASSERT(exp) _Static_assert (exp, "")
#endif
namespace DucoAPI {
using namespace apache::thrift;
using namespace apache::thrift::protocol;
using namespace apache::thrift::transport;


DucoCobot::DucoCobot(std::string ip, unsigned int port)
{
    _serverIP = ip;
    _socket = std::make_shared<TSocket>(ip, port);
    _socket->setConnTimeout(3000);
    _socket->setKeepAlive(true);
    if(port > 9000)
        _transport = std::make_shared<TFramedTransport>(_socket);
    else
        _transport = std::make_shared<TBufferedTransport>(_socket);
    _protocol = std::make_shared<TBinaryProtocol>(_transport);
    _robot = std::make_shared<RPCRobotClient>(_protocol);
}

int32_t DucoCobot::open()
{
    try {
        _transport->open();
    } catch (const std::exception& e) {
        return -1;
    }
    return 0;
}

int32_t DucoCobot::close()
{
    try {
        _transport->close();
    } catch (const std::exception& e) {
        return -1;
    }
    return 0;
}


int32_t DucoCobot::power_on(bool block)
{
    return _robot->power_on(block);
}

int32_t DucoCobot::power_off(bool block)
{
    return _robot->power_off(block);
}

int32_t DucoCobot::enable(bool block)
{
    return _robot->enable(block);
}

int32_t DucoCobot::disable(bool block)
{
    return _robot->disable(block);
}

int32_t DucoCobot::shutdown(bool block)
{
    return _robot->shutdown(block);
}

int32_t DucoCobot::stop(bool block)
{
    return _robot->stop(block);
}

int32_t DucoCobot::pause(bool block)
{
    return _robot->pause(block);
}

int32_t DucoCobot::resume(bool block)
{
    return _robot->resume(block);
}

int32_t DucoCobot::get_noneblock_taskstate(int32_t id)
{
    return _robot->get_noneblock_taskstate(id);
}

int32_t DucoCobot::set_tool_data(const std::string & name, const std::vector<double>& tool_offset, const std::vector<double>& payload, const std::vector<double>& inertia_tensor)
{
    return _robot->set_tool_data(name,tool_offset,payload,inertia_tensor);
}

int32_t DucoCobot::set_tool_data_workcell(const std::string &name, const std::vector<double> &tool, const std::vector<double> &payload, const std::vector<double> &inertia_tensor)
{
    return _robot->set_tool_data_workcell(name,tool,payload,inertia_tensor);
}

void DucoCobot::get_tcp_offset(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_tcp_offset(_return);
}

int32_t DucoCobot::set_wobj(const std::string &name, const std::vector<double> &wobj)
{
    return _robot->set_wobj(name, wobj);
}

int32_t DucoCobot::set_wobj_workcell(const std::string &name, const std::vector<double> &wobj)
{
    return _robot->set_wobj_workcell(name, wobj);
}

int32_t DucoCobot::set_wobj_offset(const std::vector<double> & wobj, bool active)
{
    return _robot->set_wobj_offset(wobj, active);
}

void DucoCobot::cal_fkine(std::vector<double>& _return, const std::vector<double>& joints_position, const std::vector<double>& tool, const std::vector<double>& wobj)
{
    _robot->cal_fkine(_return, joints_position, tool, wobj);
}

void DucoCobot::cal_ikine(std::vector<double>& _return, const std::vector<double>& p, const std::vector<double>& q_near, const std::vector<double>& tool, const std::vector<double>& wobj)
{
    _robot->cal_ikine(_return, p, q_near, tool, wobj);
}

int32_t DucoCobot::movej(const std::vector<double> & joints_list, const double v, const double a, const double r, const bool block, const bool def_acc)
{
    return _robot->movej(joints_list,v,a,r,block,def_acc);
}

int32_t DucoCobot::movej2(const std::vector<double>& joints_list, double v, double a, double r, bool block, bool def_acc)
{
    return _robot->movej2(joints_list,v,a,r,block, def_acc);
}

int32_t DucoCobot::movej_pose(const std::vector<double>& p, double v, double a, double r, const std::vector<double>& q_near, const std::string & tool, const std::string & wobj, bool block, bool def_acc)
{
    return _robot->movej_pose(p,v,a,r,q_near,tool,wobj,block,def_acc);
}

int32_t DucoCobot::movej_pose2(const std::vector<double>& p, double v, double a, double r, const std::vector<double>& q_near, const std::string & tool, const std::string & wobj, bool block, bool def_acc)
{
    return _robot->movej_pose2(p,v,a,r,q_near,tool,wobj,block,def_acc);
}

int32_t DucoCobot::movel(const std::vector<double> & p, double v, double a, double r, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc)
{
    return _robot->movel(p,v,a,r,q_near,tool,wobj,block,def_acc);
}

int32_t DucoCobot::movec(const std::vector<double> & p1, const std::vector<double> & p2, const double v, const double a, const double r, const int32_t mode, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, const bool block, const bool def_acc)
{
    return _robot->movec(p1, p2, v, a, r,mode, q_near, tool, wobj, block,def_acc);
}

int32_t DucoCobot::move_circle(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double r, int32_t mode, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc)
{
    return _robot->move_circle(p1, p2, v, a, r, mode, q_near, tool, wobj, block, def_acc);
}

int32_t DucoCobot::tcp_move(const std::vector<double> & pose_offset, double v, double a, double r, const std::string& tool, bool block, bool def_acc)
{
    return _robot->tcp_move(pose_offset,v,a,r,tool,block,def_acc);
}

int32_t DucoCobot::tcp_move_2p(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double r, const std::string& tool, const std::string& wobj, bool block, bool def_acc)
{
    return _robot->tcp_move_2p(p1,p2,v,a,r,tool,wobj,block,def_acc);
}

int32_t DucoCobot::wobj_move(const std::vector<double> &pose_offset, double v, double a, double r, const std::string &wobj, bool block, bool def_acc)
{
    return _robot->wobj_move(pose_offset, v, a, r, wobj, block, def_acc);
}

int32_t DucoCobot::wobj_move_2p(const std::vector<double> &p1, const std::vector<double> &p2, double v, double a, double r, const std::string &tool, const std::string &wobj, bool block, bool def_acc)
{
    return _robot->wobj_move_2p(p1, p2, v, a, r, tool, wobj, block, def_acc);
}

int32_t DucoCobot::speedj(const std::vector<double>& joints_list, double a, int32_t time, bool block)
{
    return _robot->speedj(joints_list, a, time, block);
}

int32_t DucoCobot::speedl(const std::vector<double>& pose_list, double a, int32_t time, bool block)
{
    return _robot->speedl(pose_list,a,time,block);
}

int32_t DucoCobot::speed_stop(bool block)
{
    return _robot->speed_stop(block);
}

int32_t DucoCobot::servoj(const std::vector<double> & joints_list, double v, double a, bool block, double kp, double kd, double smooth_vel, double smooth_acc)
{
    return _robot->servoj(joints_list,v,a,block,kp,kd,smooth_vel,smooth_acc);
}

int32_t DucoCobot::servoj_pose(const std::vector<double> & pose_list, double v, double a, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, double kp, double kd, double smooth_vel, double smooth_acc)
{
    return _robot->servoj_pose(pose_list, v, a, q_near, tool, wobj, block, kp, kd, smooth_vel, smooth_acc);
}

int32_t DucoCobot::servo_tcp(const std::vector<double> &pose_offset, double v, double a, const std::string &tool, bool block, double kp, double kd, double smooth_vel, double smooth_acc)
{
    return _robot->servo_tcp(pose_offset, v, a, tool, block, kp, kd, smooth_vel, smooth_acc);
}

int32_t DucoCobot::servol(const std::vector<double> &pose_list, double v, double a, const std::vector<double> &q_near, const std::string &tool, const std::string &wobj, bool block, double kp, double kd, double smooth_vel, double smooth_acc)
{
    return _robot->servol(pose_list, v, a, q_near, tool, wobj, block, kp, kd, smooth_vel, smooth_acc);
}

void DucoCobot::log_info(const std::string & message)
{
    _robot->log_info(message);
}

void DucoCobot::log_error(const std::string & message)
{
    _robot->log_error(message);
}

void DucoCobot::get_last_error(std::vector<std::string> & _return)
{
    _robot->get_last_error(_return);
}

int32_t DucoCobot::simulation(bool sim, bool block)
{
    return _robot->simulation(sim, block);
}

int32_t DucoCobot::speed(double val)
{
    return _robot->speed(val);
}

void DucoCobot::get_robot_state(std::vector<int8_t>& _return)
{
    _return.clear();
    _robot->get_robot_state(_return);
}

void DucoCobot::get_flange_pose(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_flange_pose(_return);
}

void DucoCobot::get_flange_speed(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_flange_speed(_return);
}

void DucoCobot::get_flange_acceleration(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_flange_acceleration(_return);
}

void DucoCobot::get_tcp_pose(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_tcp_pose(_return);
}

void DucoCobot::get_tcp_speed(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_tcp_speed(_return);
}

void DucoCobot::get_tcp_acceleration(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_tcp_acceleration(_return);
}

void DucoCobot::get_tcp_force(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_tcp_force(_return);
}

void DucoCobot::get_tcp_force_tool(std::vector<double> &_return, const std::string &tool)
{
    _return.clear();
    _robot->get_tcp_force_tool(_return, tool);
}

void DucoCobot::get_actual_joints_position(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_actual_joints_position(_return);
}

void DucoCobot::get_target_joints_position(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_target_joints_position(_return);
}

void DucoCobot::get_actual_joints_speed(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_actual_joints_speed(_return);
}

void DucoCobot::get_target_joints_speed(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_target_joints_speed(_return);
}

void DucoCobot::get_actual_joints_acceleration(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_actual_joints_acceleration(_return);
}

void DucoCobot::get_target_joints_acceleration(std::vector<double>& _return)
{
    _return.clear();
    _robot->get_target_joints_acceleration(_return);
}

void DucoCobot::get_actual_joints_torque(std::vector<double> & _return)
{
    _return.clear();
    _robot->get_actual_joints_torque(_return);
}

void DucoCobot::get_target_joints_torque(std::vector<double> & _return)
{
    _robot->get_target_joints_torque(_return);
}

int32_t DucoCobot::collision_detect(int32_t value)
{
    return _robot->collision_detect(value);
}

int32_t DucoCobot::set_load_data(const std::vector<double> & value)
{
    return _robot->set_load_data(value);
}


int32_t DucoCobot::start_realtime_mode(int32_t mode, double filter_bandwidth, double com_lost_time)
{
    return _robot->start_realtime_mode(mode, filter_bandwidth, com_lost_time);
}

int32_t DucoCobot::end_realtime_mode()
{
    return _robot->end_realtime_mode();
}

int32_t DucoCobot::realtime_data_enqueue(const std::vector<RealTimeData> &realtime_data, bool block)
{
    std::vector<RealTimeControlData> realtime_data_tmp;
    if(realtime_data.size() == 0)
        return 4;
    for (std::size_t i = 0; i < realtime_data.size(); ++i) {
        RealTimeControlData real_data;

        for (std::size_t j = 0; j < realtime_data[i].joint_pos_cmd.size(); ++j) {
            real_data.joint_pos_cmd.push_back(realtime_data[i].joint_pos_cmd[j]);
        }
        for (std::size_t j = 0; j < realtime_data[i].joint_vel_cmd.size(); ++j) {
            real_data.joint_vel_cmd.push_back(realtime_data[i].joint_vel_cmd[j]);
        }
        for (std::size_t j = 0; j < realtime_data[i].joint_torq_cmd.size(); ++j) {
            real_data.joint_torq_cmd.push_back(realtime_data[i].joint_torq_cmd[j]);
        }
        for (std::size_t k = 0; k < realtime_data[i].cart_pos_tool_wobj_cmd.size(); ++k) {
            real_data.cart_pos_tool_wobj_cmd.push_back(realtime_data[i].cart_pos_tool_wobj_cmd[k]);
        }
        for (std::size_t k = 0; k < realtime_data[i].cart_vel_tool_wobj_cmd.size(); ++k) {
            real_data.cart_vel_tool_wobj_cmd.push_back(realtime_data[i].cart_vel_tool_wobj_cmd[k]);
        }
        for (std::size_t k = 0; k < realtime_data[i].cart_ft_cmd.size(); ++k) {
            real_data.cart_ft_cmd.push_back(realtime_data[i].cart_ft_cmd[k]);
        }

        real_data.status = realtime_data[i].status;
        realtime_data_tmp.push_back(real_data);
    }

    return _robot->realtime_data_enqueue(realtime_data_tmp, block);
}

int32_t DucoCobot::clear_realtime_data_queue()
{
    return _robot->clear_realtime_data_queue();
}

int32_t DucoCobot::get_realtime_data_queue_size()
{
    return _robot->get_realtime_data_queue_size();
}

int32_t DucoCobot::trackEnqueue(const std::vector<std::vector<double> > &track, bool block)
{
    return _robot->trackEnqueue(track, block);
}

int32_t DucoCobot::trackClearQueue()
{
    return _robot->trackClearQueue();
}

int32_t DucoCobot::getQueueSize()
{
    return _robot->getQueueSize();
}

int32_t DucoCobot::trackJointMotion(double speed, double acc, bool block)
{
    return _robot->trackJointMotion(speed, acc, block);
}

int32_t DucoCobot::trackCartMotion(double speed, double acc, bool block, const std::string &tool, const std::string &wobj, double radius)
{
    return _robot->trackCartMotion(speed, acc, block, tool, wobj, radius);
}

void DucoCobot::rpc_heartbeat(int32_t time)
{
    _robot->rpc_heartbeat(time);
}

void DucoCobot::tool_read_raw_data_485(std::vector<int8_t>& _return, int32_t len)
{
    _robot->tool_read_raw_data_485(_return, len);
}

bool DucoCobot::tool_write_raw_data_485(const std::vector<int8_t>& data)
{
    return _robot->tool_write_raw_data_485(data);
}

bool DucoCobot::tool_write_raw_data_485_h(const std::vector<int8_t> & data, const std::vector<int8_t> & head)
{
    return _robot->tool_write_raw_data_485_h(data, head);
}

bool DucoCobot::tool_write_raw_data_485_ht(const std::vector<int8_t> & data, const std::vector<int8_t> & head, const std::vector<int8_t> & tail)
{
    return _robot->tool_write_raw_data_485_ht(data, head, tail);
}

void DucoCobot::tool_read_raw_data_485_h(std::vector<int8_t> & _return, const std::vector<int8_t> & head, int32_t len)
{
    _robot->tool_read_raw_data_485_h(_return, head, len);
}

void DucoCobot::tool_read_raw_data_485_ht(std::vector<int8_t> & _return, const std::vector<int8_t> & head, const std::vector<int8_t> & tail)
{
    _robot->tool_read_raw_data_485_ht(_return, head, tail);
}

void DucoCobot::get_system_value_lists(std::vector<std::vector<double> > &value, const std::string &name)
{
    _robot->get_system_value_lists(value, name);
}

int32_t DucoCobot::fc_start(int32_t mode)
{
    return _robot->fc_start(mode);
}

int32_t DucoCobot::fc_stop()
{
    return _robot->fc_stop();
}

int32_t DucoCobot::fc_config(const std::vector<bool> & direction, const std::vector<double> & ref_ft, const std::vector<double> & mass,
                             const std::vector<double> & stiff, const std::vector<double> & damp, const std::vector<double> & max_vel, const std::vector<double> & dead_zone,
                             const std::string& toolname, const std::string& wobjname, const int32_t type, const int32_t mode)
{
    return _robot->fc_config(direction, ref_ft, mass, stiff, damp, max_vel, dead_zone, toolname, wobjname, type, mode);
}

int32_t DucoCobot::fc_move(bool block)
{
    return _robot->fc_move(block);
}

int32_t DucoCobot::fc_guard_act(const std::vector<bool> & direction, const std::vector<double> & ref_ft, const std::string& toolname, const std::string& wobjname, int32_t type, int32_t force_property)
{
    return _robot->fc_guard_act(direction, ref_ft, toolname, wobjname, type, force_property);
}

int32_t DucoCobot::fc_guard_deact()
{
    return _robot->fc_guard_deact();
}

int32_t DucoCobot::fc_force_set_value(const std::vector<bool> & direction, const std::vector<double> & ref_ft)
{
    return _robot->fc_force_set_value(direction, ref_ft);
}

int32_t DucoCobot::fc_wait_pos(const std::vector<double> & middle, const std::vector<double> & range, bool absolute, int32_t duration, int32_t timeout)
{
    return _robot->fc_wait_pos(middle, range, absolute, duration, timeout);
}

int32_t DucoCobot::fc_wait_vel(const std::vector<double> & middle, const std::vector<double> & range, bool absolute, int32_t duration, int32_t timeout)
{
    return _robot->fc_wait_vel(middle, range, absolute, duration, timeout);
}

int32_t DucoCobot::fc_wait_ft(const std::vector<double> & middle, const std::vector<double> & range, bool absolute, int32_t duration, int32_t timeout)
{
    return _robot->fc_wait_ft(middle, range, absolute, duration, timeout);
}

int32_t DucoCobot::fc_wait_logic(const std::vector<int32_t> & value)
{
    return _robot->fc_wait_logic(value);
}

void DucoCobot::fc_get_ft(std::vector<double> & _return)
{
    _robot->fc_get_ft(_return);
}

bool DucoCobot::fc_mode_is_active()
{
    return _robot->fc_mode_is_active();
}

int32_t DucoCobot::move_jog(const MoveJogTaskParam &param, bool block)
{
    ::MoveJogTaskParam jog_param;
    jog_param.jog_direction = param.jog_direction;
    jog_param.jog_type = param.jog_type;
    jog_param.axis_num = param.axis_num;
    jog_param.vel = param.vel;
    jog_param.jog_coordinate = param.jog_coordinate;
    jog_param.use_step = param.use_step;
    jog_param.step_cartvalue = param.step_cartvalue;
    jog_param.step_jointValue = param.step_jointValue;
    return _robot->move_jog(jog_param, block);
}

int32_t DucoCobot::stop_manual_move(bool block)
{
    return _robot->stop_manual_move(block);
}

void DucoCobot::get_robot_version(std::string &_return)
{
    _robot->get_robot_version(_return);
}

int32_t DucoCobot::get_global_speed()
{
    return _robot->get_global_speed();
}

std::string DucoCobot::get_version()
{
    std::string version("3.8.5");
    return version;
}

void DucoCobot::get_robot_status(RobotStatus &status)
{
    ::RobotStatus robot_status;
    status.actual_joint_pos.clear();
    status.actual_joint_vel.clear();
    status.actual_joint_acc.clear();
    status.actual_joint_torq.clear();
    status.actual_joint_temp.clear();
    status.actual_joint_current.clear();
    status.auxiliary_joint_vel.clear();
    status.auxiliary_joint_acc.clear();
    status.auxiliary_joint_torq.clear();
    status.target_joint_pos.clear();
    status.target_joint_vel.clear();
    status.target_joint_acc.clear();
    status.target_joint_torq.clear();
    status.servo_errid.clear();
    status.servo_statusword.clear();
    status.actual_tcp_pose.clear();
    status.actual_tcp_vel.clear();
    status.actual_tcp_acc.clear();
    status.target_tcp_pose.clear();
    status.target_tcp_vel.clear();
    status.target_tcp_acc.clear();
    _robot->get_robot_status(robot_status);

    status.actual_joint_pos = robot_status.actual_joint_pos;
    status.actual_joint_vel = robot_status.actual_joint_vel;
    status.actual_joint_acc = robot_status.actual_joint_acc;
    status.actual_joint_torq = robot_status.actual_joint_torq;
    status.actual_joint_temp = robot_status.actual_joint_temp;
    status.actual_joint_current = robot_status.actual_joint_current;
    status.auxiliary_joint_vel = robot_status.auxiliary_joint_vel;
    status.auxiliary_joint_acc = robot_status.auxiliary_joint_acc;
    status.auxiliary_joint_torq = robot_status.auxiliary_joint_torq;
    status.target_joint_pos = robot_status.target_joint_pos;
    status.target_joint_vel = robot_status.target_joint_vel;
    status.target_joint_acc = robot_status.target_joint_acc;
    status.target_joint_torq = robot_status.target_joint_torq;
    status.servo_errid = robot_status.servo_errid;
    status.servo_statusword = robot_status.servo_statusword;
    status.actual_tcp_pose = robot_status.actual_tcp_pose;
    status.actual_tcp_vel = robot_status.actual_tcp_vel;
    status.actual_tcp_acc = robot_status.actual_tcp_acc;
    status.target_tcp_pose = robot_status.target_tcp_pose;
    status.target_tcp_vel = robot_status.target_tcp_vel;
    status.target_tcp_acc = robot_status.target_tcp_acc;
}

void DucoCobot::get_tcp_pose_coord(std::vector<double> &_return, const std::string &tool, const std::string &wobj)
{
    _return.clear();
    _robot->get_tcp_pose_coord(_return, tool, wobj);
}

int32_t DucoCobot::set_servo_config(int32_t axis_num, int32_t id, int32_t value, int32_t qfmt, bool block)
{
    return _robot->set_servo_config(axis_num, id, value, qfmt, block);
}

int32_t DucoCobot::apply_servo_config(int32_t axis_num, bool block)
{
    return _robot->apply_servo_config(axis_num, block);
}

void DucoCobot::get_motor_pole_pair_number(std::vector<int16_t> &_return)
{
    _robot->get_motor_pole_pair_number(_return);
}

void DucoCobot::get_motor_stator_slots(std::vector<int16_t> &_return)
{
    _robot->get_motor_stator_slots(_return);
}

void DucoCobot::get_axis_ratio(std::vector<int16_t> &_return)
{
    _robot->get_axis_ratio(_return);
}

int32_t DucoCobot::collision_detection_reset()
{
    return _robot->collision_detection_reset();
}

int32_t DucoCobot::set_servo_file_params(int32_t axis_num, int32_t id, const std::string &name, double value, double qfmt)
{
    return _robot->set_servo_file_params(axis_num, id, name, value, qfmt);
}

int32_t DucoCobot::enable_eaxis(const std::string &name, int32_t axis_num)
{
    return _robot->enable_eaxis(name, axis_num);
}

int32_t DucoCobot::disable_eaxis(const std::string &name, int32_t axis_num)
{
    return _robot->disable_eaxis(name, axis_num);
}

int32_t DucoCobot::move_eaxis(const std::string& name, const std::vector<double> & epose, const std::vector<double> & v, const bool block)
{
    return _robot->move_eaxis(name, epose, v, block);
}

int32_t DucoCobot::movel_eaxis(const std::string& name, const std::vector<double> & p, const double v, const double a, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, const bool block)
{
    return _robot->movel_eaxis(name, p, v, a, q_near, tool, wobj, block);
}

int32_t DucoCobot::servo_eaxis(const std::string& name, const std::vector<double> & epose, const std::vector<double> & v, const std::vector<double> & a,
                               const bool block, const double kp, const double kd, const double smooth_vel, const double smooth_acc)
{
    return _robot->servo_eaxis(name, epose, v, a, block, kp, kd, smooth_vel, smooth_acc);
}

int32_t DucoCobot::servol_eaxis(const std::string& name, const std::vector<double> & p, const double v, const double a, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, const bool block,
                                const double kp, const double kd, const double smooth_vel, const double smooth_acc)
{
    return _robot->servol_eaxis(name, p, v, a, q_near, tool, wobj, block, kp, kd, smooth_vel, smooth_acc);
}

int32_t DucoCobot::switch_mode(const int32_t mode)
{
    return _robot->switch_mode(mode);
}

void DucoCobot::get_origin_DH(std::vector<std::vector<double> > &dh)
{
    return _robot->get_origin_DH(dh);
}

void DucoCobot::get_calib_DH(std::vector<std::vector<double> > &calib_dh)
{
    return _robot->get_calib_DH(calib_dh);
}

void DucoCobot::get_robot_type(std::vector<std::string> &type)
{
    _robot->get_robot_type(type);
}

void DucoCobot::get_ext_torque(std::vector<double> &torque)
{
    _robot->get_ext_torque(torque);
}

int32_t DucoCobot::set_kinematic_calibration_params(const std::vector<std::vector<double> > &params)
{
    return _robot->set_kinematic_calibration_params(params);
}

int32_t DucoCobot::set_stiffness_calibration_params(const std::vector<std::vector<double> > &params)
{
    return _robot->set_stiffness_calibration_params(params);
}

void DucoCobot::get_stiffness_calibration_params(std::vector<std::vector<double> > &params)
{
    return _robot->get_stiffness_calibration_params(params);
}

int32_t DucoCobot::save_stiffness_calibration_params(const std::string &passwd)
{
    return _robot->save_stiffness_calibration_params(passwd);
}

void DucoCobot::get_joint_motion_params(std::vector<std::vector<double> > &params)
{
    return _robot->get_joint_motion_params(params);
}

void DucoCobot::get_pos_bias(std::vector<double> &bias)
{
    _robot->get_pos_bias(bias);
}

int32_t DucoCobot::set_dynamic_calibration_params(const std::vector<std::vector<double> > &params)
{
    return _robot->set_dynamic_calibration_params(params);
}

void DucoCobot::get_dynamic_calibration_params(std::vector<std::vector<double> > &params)
{
    _robot->get_dynamic_calibration_params(params);
}

int32_t DucoCobot::set_dynamic_bias_params(const std::vector<bool> &active, const std::vector<double> &bias_param)
{
    return _robot->set_dynamic_bias_params(active, bias_param);
}

int32_t DucoCobot::set_dynamic_calibration_minparams(const std::vector<double> &min_params, const std::vector<double> &g)
{
    return _robot->set_dynamic_calibration_minparams(min_params, g);
}

void DucoCobot::get_dynamic_calibration_minparams(std::vector<double> &_return)
{
    _robot->get_dynamic_calibration_minparams(_return);
}

void DucoCobot::get_dynamic_g(std::vector<double> &_return)
{
    _robot->get_dynamic_g(_return);
}

int32_t DucoCobot::upload_robot_param_to_toolboard(const std::string &passwd)
{
    return _robot->upload_robot_param_to_toolboard(passwd);
}

void DucoCobot::get_joint_gravtorque(std::vector<double> &_return)
{
    return _robot->get_joint_gravtorque(_return);
}

void DucoCobot::get_auxiliary_position(std::vector<double> &_return)
{
    return _robot->get_auxiliary_position(_return);
}

void DucoCobot::get_joint_dynamic_torque(std::vector<double> &_return)
{
    return _robot->get_joint_dynamic_torque(_return);
}

bool DucoCobot::get_collision_state()
{
    return _robot->get_collision_state();
}

int32_t DucoCobot::get_collision_num()
{
    return _robot->get_collision_num();
}

int32_t DucoCobot::save_dynamic_calibration_minparams(const std::string &passwd)
{
    return _robot->save_dynamic_calibration_minparams(passwd);
}

int32_t DucoCobot::set_kinematic_calibration_info(const std::string &passwd, const std::string& version)
{
    return _robot->set_kinematic_calibration_info(passwd, version);
}

int32_t DucoCobot::set_dynamic_calibration_info(const std::string &passwd, const std::string& version)
{
    return _robot->set_dynamic_calibration_info(passwd, version);
}

int32_t DucoCobot::set_vibration_calibration_info(const std::string &passwd, const std::string& info)
{
    return _robot->set_vibration_calibration_info(passwd, info);
}

void DucoCobot::get_axis_motor_rated_current(std::vector<double> &rated)
{
    _robot->get_axis_motor_rated_current(rated);
}

void DucoCobot::get_axis_motor_kt(std::vector<double> &kt)
{
    _robot->get_axis_motor_kt(kt);
}

void DucoCobot::get_vibration_calibration_params(std::vector<std::vector<double> > &params)
{
    _robot->get_vibration_calibration_params(params);
}

int32_t DucoCobot::save_kinematic_calibration_params(const std::string &passwd)
{
    return _robot->save_kinematic_calibration_params(passwd);
}

int32_t DucoCobot::save_dynamic_calibration_params(const std::string &passwd)
{
    return _robot->save_dynamic_calibration_params(passwd);
}

int32_t DucoCobot::set_vibration_params(const bool vibration_on, const int32_t vibration_mode)
{
    return _robot->set_vibration_params(vibration_on, vibration_mode);
}

bool DucoCobot::get_vibration_on()
{
    return _robot->get_vibration_on();
}

int32_t DucoCobot::get_vibration_mode()
{
    return _robot->get_vibration_mode();
}

void DucoCobot::get_eaxis_status(RobotStatus &status, const std::string &name)
{
    ::RobotStatus robot_status;
    status.actual_joint_pos.clear();
    status.actual_joint_vel.clear();
    status.actual_joint_acc.clear();
    status.actual_joint_torq.clear();
    status.actual_joint_temp.clear();
    status.actual_joint_current.clear();
    status.auxiliary_joint_vel.clear();
    status.auxiliary_joint_acc.clear();
    status.auxiliary_joint_torq.clear();
    status.target_joint_pos.clear();
    status.target_joint_vel.clear();
    status.target_joint_acc.clear();
    status.target_joint_torq.clear();
    status.servo_errid.clear();
    status.servo_statusword.clear();
    status.actual_tcp_pose.clear();
    status.actual_tcp_vel.clear();
    status.actual_tcp_acc.clear();
    status.target_tcp_pose.clear();
    status.target_tcp_vel.clear();
    status.target_tcp_acc.clear();
    _robot->get_eaxis_status(robot_status, name);

    status.actual_joint_pos = robot_status.actual_joint_pos;
    status.actual_joint_vel = robot_status.actual_joint_vel;
    status.actual_joint_acc = robot_status.actual_joint_acc;
    status.actual_joint_torq = robot_status.actual_joint_torq;
    status.actual_joint_temp = robot_status.actual_joint_temp;
    status.actual_joint_current = robot_status.actual_joint_current;
    status.auxiliary_joint_vel = robot_status.auxiliary_joint_vel;
    status.auxiliary_joint_acc = robot_status.auxiliary_joint_acc;
    status.auxiliary_joint_torq = robot_status.auxiliary_joint_torq;
    status.target_joint_pos = robot_status.target_joint_pos;
    status.target_joint_vel = robot_status.target_joint_vel;
    status.target_joint_acc = robot_status.target_joint_acc;
    status.target_joint_torq = robot_status.target_joint_torq;
    status.servo_errid = robot_status.servo_errid;
    status.servo_statusword = robot_status.servo_statusword;
    status.actual_tcp_pose = robot_status.actual_tcp_pose;
    status.actual_tcp_vel = robot_status.actual_tcp_vel;
    status.actual_tcp_acc = robot_status.actual_tcp_acc;
    status.target_tcp_pose = robot_status.target_tcp_pose;
    status.target_tcp_vel = robot_status.target_tcp_vel;
    status.target_tcp_acc = robot_status.target_tcp_acc;
}

int32_t DucoCobot::set_tool_digital_out(int16_t num, bool value, bool block)
{
    return _robot->set_tool_digital_out(num,value,block);
}

bool DucoCobot::get_tool_digital_in(int16_t num)
{
    return _robot->get_tool_digital_in(num);
}

bool DucoCobot::get_tool_digital_out(int16_t num)
{
    return _robot->get_tool_digital_out(num);
}

void DucoCobot::get_tool_load(std::vector<double> & load)
{
    return _robot->get_tool_load(load);
}

void DucoCobot::get_wobj(std::vector<double> & wobj)
{
    return _robot->get_wobj(wobj);
}

int32_t DucoCobot::set_installation(const std::vector<double> &installation)
{
    return _robot->set_installation(installation);
}

int32_t DucoCobot::fc_set_mode(const int32_t mode)
{
    return _robot->fc_set_mode(mode);
}

int32_t DucoCobot::fc_config_joint(const std::vector<bool> &direction, const std::vector<double> &stiff, const std::vector<double> &damp)
{
    return _robot->fc_config_joint(direction, stiff, damp);
}

int32_t DucoCobot::fc_config_cart(const std::vector<bool> &direction, const std::vector<double> &ref_ft, const std::vector<double> &stiff, const std::vector<double> &damp, const std::vector<double> &number_list, const std::string &tool, const std::string &wobj, const int32_t value)
{
    return _robot->fc_config_cart(direction, ref_ft, stiff, damp, number_list, tool, wobj, value);
}

int32_t DucoCobot::teach_mode(bool block)
{
    return _robot->teach_mode(block);
}

int32_t DucoCobot::end_teach_mode(bool block)
{
    return _robot->end_teach_mode(block);
}

int32_t DucoCobot::enable_servo_optimization(bool vel_limit, bool pos_limit, bool kine_limit)
{
    return _robot->enable_servo_optimization(vel_limit, pos_limit, kine_limit);
}

int32_t DucoCobot::disable_servo_optimization()
{
    return _robot->disable_servo_optimization();
}

}



