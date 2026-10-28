#ifndef DUCO_COBOT_WAIST_H
#define DUCO_COBOT_WAIST_H

#include <cstdint>
#include "DucoCobot.h"

class RPCWaistClient;

namespace DucoAPI {

/**
 * @brief 腰部机器人 RPC 接口，方法及默认参数参考 Python SDK。
 * @note 复用 DucoCobot.h 中的 RobotStatus、TaskState 等公共类型。
 * 每个对象拥有独立连接；同一对象不可在多个线程中并发调用 RPC。
 * 除 open()/close() 外，通信异常向调用者抛出；任务返回码由控制器提供。
 */
class DucoCobotWaist {
public:
    /**
     * @brief 创建腰部连接，尚未连接控制器。
     * @param ip 控制器 IP 或主机名。
     * @param port 腰部服务端口，Python 示例为 7020。
     * @note 端口大于 9000 使用帧传输，否则使用缓冲传输；连接超时 3000 ms。
     */
    DucoCobotWaist(std::string ip, unsigned int port);
    /** @brief 释放连接，不自动停止任务或下使能。 */
    ~DucoCobotWaist();
    DucoCobotWaist(const DucoCobotWaist&) = delete;
    DucoCobotWaist& operator=(const DucoCobotWaist&) = delete;

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

    /**
     * @brief 获取腰部法兰在基坐标系下的位姿 [x,y,z,rx,ry,rz]。
     * @param[out] values 法兰位姿 [x,y,z,rx,ry,rz]，单位 m、rad。
     */
    void get_flange_pose(std::vector<double>& values);

    /**
     * @brief 获取腰部法兰在基坐标系下的速度。
     * @param[out] values 法兰速度，单位 m/s、rad/s。
     */
    void get_flange_speed(std::vector<double>& values);

    /**
     * @brief 获取腰部法兰在基坐标系下的加速度。
     * @param[out] values 法兰加速度，单位 m/s²、rad/s²。
     */
    void get_flange_acceleration(std::vector<double>& values);

    /**
     * @brief 计算腰部正运动学。
     * @param[out] pose 工件坐标系下的 TCP 位姿 [x,y,z,rx,ry,rz]，单位 m、rad。
     * @param joints_position 待计算的关节角，单位 rad。
     * @param tool TCP 偏移量 [x,y,z,rx,ry,rz]，单位 m、rad，空向量使用当前工具。
     * @param wobj 工件相对基坐标系的位姿，单位 m、rad，空向量使用当前工件。
     */
    void cal_fkine(std::vector<double>& pose, const std::vector<double>& joints_position, const std::vector<double>& tool, const std::vector<double>& wobj);

    /**
     * @brief 计算腰部逆运动学，选择靠近参考关节角的解。
     * @param[out] joints 求得的关节角列表，单位 rad。
     * @param p 工件坐标系下的目标 TCP 位姿，单位 m、rad。
     * @param q_near 参考关节角，单位 rad，空向量使用当前关节角。
     * @param tool TCP 偏移量 [x,y,z,rx,ry,rz]，单位 m、rad，空向量使用当前工具。
     * @param wobj 工件相对基坐标系的位姿，单位 m、rad，空向量使用当前工件。
     */
    void cal_ikine(std::vector<double>& joints, const std::vector<double>& p, const std::vector<double>& q_near, const std::vector<double>& tool, const std::vector<double>& wobj);

    /**
     * @brief 腰部通过关节空间运动到目标位姿。
     * @param p 工件坐标系下目标位姿 [x,y,z,rx,ry,rz]，单位 m、rad。
     * @param v 关节速度百分比，范围 (0,100]。
     * @param a 关节加速度百分比，范围 (0,100]。
     * @param r 融合半径百分比，范围 [0,50)，0 表示无融合。
     * @param q_near 逆解参考关节角，单位 rad，空向量使用当前关节角。
     * @param tool 工具名称，空字符串使用当前工具。
     * @param wobj 工件坐标系名称，空字符串使用当前工件。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     * @param def_acc true 使用系统默认加速度，默认 false。
     */
    int32_t movej_pose(const std::vector<double>& p, double v, double a, double r, const std::vector<double>& q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc = false);

    /**
     * @brief 腰部通过关节空间运动到目标位姿。
     * @param p 工件坐标系下目标位姿 [x,y,z,rx,ry,rz]，单位 m、rad。
     * @param v 关节角速度，单位 rad/s。
     * @param a 关节角加速度，单位 rad/s²。
     * @param r 融合半径，单位 m，0 表示无融合。
     * @param q_near 逆解参考关节角，单位 rad，空向量使用当前关节角。
     * @param tool 工具名称，空字符串使用当前工具。
     * @param wobj 工件坐标系名称，空字符串使用当前工件。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     * @param def_acc true 使用系统默认加速度，默认 false。
     */
    int32_t movej_pose2(const std::vector<double>& p, double v, double a, double r, const std::vector<double>& q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc = false);

    /**
     * @brief 腰部末端沿直线路径移动到目标位姿。
     * @param p 工件坐标系下目标位姿 [x,y,z,rx,ry,rz]，单位 m、rad。
     * @param v 末端速度，单位 m/s。
     * @param a 末端加速度，单位 m/s²。
     * @param r 融合半径，单位 m，0 表示无融合。
     * @param q_near 逆解参考关节角，单位 rad，空向量使用当前关节角。
     * @param tool 工具名称，空字符串使用当前工具。
     * @param wobj 工件坐标系名称，空字符串使用当前工件。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     * @param def_acc true 使用系统默认加速度，默认 false。
     */
    int32_t movel(const std::vector<double>& p, double v, double a, double r, const std::vector<double>& q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc = false);

    /**
     * @brief 腰部末端沿工具坐标系移动指定增量。
     * @param pose_offset 位姿增量 [x,y,z,rx,ry,rz]，单位 m、rad。
     * @param v 直线速度，单位 m/s；平移分量全零时按比例换算为角速度。
     * @param a 加速度，单位 m/s²。
     * @param r 融合半径，单位 m，0 表示无融合。
     * @param tool 工具名称，空字符串使用当前工具。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     * @param def_acc true 使用系统默认加速度，默认 false。
     */
    int32_t tcp_move(const std::vector<double>& pose_offset, double v, double a, double r, const std::string& tool, bool block, bool def_acc = false);

    /**
     * @brief 腰部末端按直线路径执行笛卡尔伺服运动。
     * @param pose_list 工件坐标系下目标 TCP 位姿，单位 m、rad。
     * @param v 末端速度，单位 m/s。
     * @param a 末端加速度，单位 m/s²。
     * @param q_near 逆解参考关节角，单位 rad，空向量使用当前关节角。
     * @param tool 工具名称，空字符串使用当前工具。
     * @param wobj 工件坐标系名称，空字符串使用当前工件。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     * @param kp 比例参数，默认 200，建议使用默认值。
     * @param kd 微分参数，默认 25，建议使用默认值。
     * @param smooth_vel 速度平滑参数，范围 [1,10]，默认 10。
     * @param smooth_acc 加速度平滑参数，范围 [0,1]，默认 1，以 Python 函数默认值为准。
     * @note 连续发送离散点时，较大的平滑参数可改善平滑度；按实际控制器能力调整。
     * @note tool、wobj 按当前 RPC 签名提供；Python 高层 servol 漏传了这两个参数。
     */
    int32_t servol(const std::vector<double>& pose_list, double v, double a, const std::vector<double>& q_near, const std::string& tool, const std::string& wobj, bool block = false, double kp = 200, double kd = 25, double smooth_vel = 10, double smooth_acc = 1);

    /**
     * @brief 腰部通过关节空间执行目标位姿伺服运动。
     * @param pose_list 工件坐标系下目标 TCP 位姿，单位 m、rad。
     * @param v 关节速度，单位 rad/s。
     * @param a 关节加速度，单位 rad/s²，与现有 C++ 关节空间伺服接口一致。
     * @param q_near 逆解参考关节角，单位 rad，空向量使用当前关节角。
     * @param tool 工具名称，空字符串使用当前工具。
     * @param wobj 工件坐标系名称，空字符串使用当前工件。
     * @param block 是否阻塞；false 时立即返回任务 ID。
     * @return 阻塞时返回任务结束状态，非阻塞时返回任务 ID。
     * @param kp 比例参数，默认 200，建议使用默认值。
     * @param kd 微分参数，默认 25，建议使用默认值。
     * @param smooth_vel 速度平滑参数，范围 [1,10]，默认 10。
     * @param smooth_acc 加速度平滑参数，范围 [0,1]，默认 1，以 Python 函数默认值为准。
     * @note 连续发送离散点时，较大的平滑参数可改善平滑度；按实际控制器能力调整。
     */
    int32_t servoj_pose(const std::vector<double>& pose_list, double v, double a, const std::vector<double>& q_near, const std::string& tool, const std::string& wobj, bool block = false, double kp = 200, double kd = 25, double smooth_vel = 10, double smooth_acc = 1);

private:
    std::shared_ptr<apache::thrift::transport::TSocket> _socket;
    std::shared_ptr<apache::thrift::transport::TTransport> _transport;
    std::shared_ptr<apache::thrift::protocol::TProtocol> _protocol;
    std::shared_ptr<RPCWaistClient> _client;
};

} // namespace DucoAPI
#endif
