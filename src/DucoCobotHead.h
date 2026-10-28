#ifndef DUCO_COBOT_HEAD_H
#define DUCO_COBOT_HEAD_H

#include <cstdint>
#include "DucoCobot.h"

class RPCHeadClient;

namespace DucoAPI {

/**
 * @brief 头部机器人 RPC 接口，方法及默认参数参考 Python SDK。
 * @note 复用 DucoCobot.h 中的 RobotStatus、TaskState 等公共类型。
 * 每个对象拥有独立连接；同一对象不可在多个线程中并发调用 RPC。
 * 除 open()/close() 外，通信异常向调用者抛出；任务返回码由控制器提供。
 */
class DucoCobotHead {
public:
    /**
     * @brief 创建头部连接，尚未连接控制器。
     * @param ip 控制器 IP 或主机名。
     * @param port 头部服务端口，Python 示例为 7021。
     * @note 端口大于 9000 使用帧传输，否则使用缓冲传输；连接超时 3000 ms。
     */
    DucoCobotHead(std::string ip, unsigned int port);
    /** @brief 释放连接，不自动停止任务或下使能。 */
    ~DucoCobotHead();
    DucoCobotHead(const DucoCobotHead&) = delete;
    DucoCobotHead& operator=(const DucoCobotHead&) = delete;

    /** @brief 连接控制器。@return 0 成功，-1 失败。 */
    int32_t open();
    /** @brief 断开连接，不自动下使能。@return 0 成功，-1 失败。 */
    int32_t close();

    /**
     * @brief 使能指定轴。
     * @param axis_num 轴参数，编号及是否支持全轴选择由控制器协议定义。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     */
    int32_t enable(int32_t axis_num, bool block);

    /**
     * @brief 下使能指定轴。
     * @param axis_num 轴参数，编号及是否支持全轴选择由控制器协议定义。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     */
    int32_t disable(int32_t axis_num, bool block);

    /**
     * @brief 停止当前部件的所有任务。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     */
    int32_t stop(bool block);

    /**
     * @brief 暂停当前部件的所有任务。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     */
    int32_t pause(bool block);

    /**
     * @brief 恢复当前部件暂停的任务。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     */
    int32_t resume(bool block);

    /**
     * @brief 获取当前部件的机器人状态。
     * @param[out] state 状态列表：机器人、程序、安全控制器、操作模式。
     */
    void get_robot_state(std::vector<int8_t>& state);

    /**
     * @brief 获取当前部件的完整状态快照。
     * @param[out] status 实际/目标关节状态、伺服错误及 TCP 状态；有效字段由控制器决定。
     */
    void get_robot_status(RobotStatus& status);

    /**
     * @brief 获取实际关节位置。
     * @param[out] values 关节位置列表，单位 rad，长度由设备关节数量决定。
     */
    void get_actual_joints_position(std::vector<double>& values);

    /**
     * @brief 获取规划目标关节位置。
     * @param[out] values 关节位置列表，单位 rad，长度由设备关节数量决定。
     */
    void get_target_joints_position(std::vector<double>& values);

    /**
     * @brief 获取实际关节速度。
     * @param[out] values 关节速度列表，单位 rad/s，长度由设备关节数量决定。
     */
    void get_actual_joints_speed(std::vector<double>& values);

    /**
     * @brief 获取规划目标关节速度。
     * @param[out] values 关节速度列表，单位 rad/s，长度由设备关节数量决定。
     */
    void get_target_joints_speed(std::vector<double>& values);

    /**
     * @brief 获取实际关节加速度。
     * @param[out] values 关节加速度列表，单位 rad/s²，长度由设备关节数量决定。
     */
    void get_actual_joints_acceleration(std::vector<double>& values);

    /**
     * @brief 获取规划目标关节加速度。
     * @param[out] values 关节加速度列表，单位 rad/s²，长度由设备关节数量决定。
     */
    void get_target_joints_acceleration(std::vector<double>& values);

    /**
     * @brief 按关节空间运动到目标关节角。
     * @param joints_list 目标关节角列表，单位 rad，长度须与部件关节数量一致。
     * @param v 关节速度百分比，范围 (0,100]。
     * @param a 关节加速度百分比，范围 (0,100]。
     * @param r 融合半径百分比，范围 [0,50)，0 表示无融合。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     * @param def_acc true 使用系统默认加速度，false 使用 a，默认 false。
     * @note 无融合正常结束为 ST_Finished；融合运动可能返回 ST_Interrupt。
     */
    int32_t movej(const std::vector<double>& joints_list, double v, double a, double r, bool block, bool def_acc = false);

    /**
     * @brief 按关节空间运动到目标关节角。
     * @param joints_list 目标关节角列表，单位 rad，长度须与部件关节数量一致。
     * @param v 关节角速度，单位 rad/s。
     * @param a 关节角加速度，单位 rad/s²。
     * @param r 融合半径，单位 m，0 表示无融合。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     * @param def_acc true 使用系统默认加速度，false 使用 a，默认 false。
     * @note 无融合正常结束为 ST_Finished；融合运动可能返回 ST_Interrupt。
     */
    int32_t movej2(const std::vector<double>& joints_list, double v, double a, double r, bool block, bool def_acc = false);

    /**
     * @brief 通过关节空间伺服运动到目标关节角。
     * @param joints_list 目标关节角，单位 rad，长度须与部件关节数量一致。
     * @param v 最大关节角速度，单位 rad/s。
     * @param a 最大关节角加速度，单位 rad/s²。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     * @param kp 比例参数，默认 200，建议使用默认值。
     * @param kd 微分参数，默认 25，建议使用默认值。
     * @param smooth_vel 速度平滑参数，范围 [1,10]，默认 10。
     * @param smooth_acc 加速度平滑参数，范围 [0,1]，默认 1，以 Python 函数默认值为准。
     * @note 连续发送离散点时，较大的平滑参数可改善平滑度；按实际控制器能力调整。
     */
    int32_t servoj(const std::vector<double>& joints_list, double v, double a, bool block = false, double kp = 200, double kd = 25, double smooth_vel = 10, double smooth_acc = 1);

    /**
     * @brief 根据非阻塞任务 ID 查询状态。
     * @param id 非阻塞接口返回的任务 ID。
     * @return 当前任务状态，枚举见 DucoAPI::TaskState。
     */
    int32_t get_noneblock_taskstate(int32_t id);

    /**
     * @brief 刷新远程连接心跳。
     * @param time 心跳保护时间，单位 ms，默认 1000。
     * @note 应使用独立连接在线程中周期调用；超时停止行为以控制器配置为准。
     */
    void rpc_heartbeat(int32_t time = 1000);

private:
    std::shared_ptr<apache::thrift::transport::TSocket> _socket;
    std::shared_ptr<apache::thrift::transport::TTransport> _transport;
    std::shared_ptr<apache::thrift::protocol::TProtocol> _protocol;
    std::shared_ptr<RPCHeadClient> _client;
};

} // namespace DucoAPI
#endif
