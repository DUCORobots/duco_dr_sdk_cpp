#ifndef _DUCOCOBOT
#define _DUCOCOBOT
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <thread>
#include <list>

namespace apache {
	namespace thrift {
		namespace transport {
			class TSocket;
			class TTransport;
		}
	}
}
namespace apache {
	namespace thrift {
		namespace protocol {
			class TProtocol;
		}
	}
}
class RPCRobotClient;
namespace DucoAPI {
/**
 * @brief 机器人状态信息
 */
enum StateRobot {
    SR_Start = 0,
    SR_Initialize = 1,
    SR_Logout = 2,
    SR_Login = 3,
    SR_PowerOff = 4,
    SR_Disable = 5,
    SR_Enable = 6
};

/**
 * @brief 程序状态信息
 */
enum StateProgram {
    SP_Stopped = 0,
    SP_Stopping = 1,
    SP_Running = 2,
    SP_Paused = 3,
    SP_Pausing = 4
};

/**
 * @brief 机器人操作模式信息
 */
enum OperationMode {
    kManual = 0,
    kAuto = 1,
    kRemote = 2
};

/**
 * @brief 任务状态信息
 */
enum TaskState {
    ST_Idle = 0,
    ST_Running = 1,
    ST_Paused = 2,
    ST_Stopped = 3,
    ST_Finished = 4,
    ST_Interrupt = 5,
    ST_Error = 6,
    ST_Illegal = 7,
    ST_ParameterMismatch = 8
};

// 机器人点动相关信息
struct MoveJogTaskParam
{
    int32_t jog_direction;  // 运动方向, 0:正方向, 1:负方向
    int32_t jog_type;       // 1: 空间点动, 2: 关节点动
    int32_t axis_num;       // 关节索引号, 0--5
    double vel;             // 速度百分比
    int32_t jog_coordinate; // 参考坐标系, 0: 世界, 1: 基座, 2: 工具, 3: 工件
    bool use_step;          // 是否步进模式, true: 步进, false: 连续
    double step_jointValue; // 关节和空间RX/RY/RZ步进弧度, 单位: rad
    double step_cartvalue;  // 空间X/Y/Z步进距离, 单位: m
};

// 机器人相关信息
struct RobotStatus{
    std::vector<double>  actual_joint_pos;       // 实际关节位置
    std::vector<double>  actual_joint_vel;       // 实际角速度
    std::vector<double>  actual_joint_acc;       // 实际角加速度
    std::vector<double>  actual_joint_torq;      // 实际力矩
    std::vector<double>  actual_joint_temp;      //
    std::vector<double>  actual_joint_current;   // 实际关节电流
    std::vector<double>  auxiliary_joint_vel;    //
    std::vector<double>  auxiliary_joint_acc;    //
    std::vector<double>  auxiliary_joint_torq;   //
    std::vector<double>  target_joint_pos;       // 目标关节位置
    std::vector<double>  target_joint_vel;       // 目标角速度
    std::vector<double>  target_joint_acc;       // 目标角加速度
    std::vector<double>  target_joint_torq;      // 目标力矩
    std::vector<int32_t> servo_errid;            // 错误ID
    std::vector<int32_t> servo_statusword;       // 状态
    std::vector<double>  actual_tcp_pose;        // 实际末端位姿
    std::vector<double>  actual_tcp_vel;         // 实际末端速度
    std::vector<double>  actual_tcp_acc;         // 实际末端加速度
    std::vector<double>  target_tcp_pose;        // 目标末端位姿
    std::vector<double>  target_tcp_vel;         // 目标末端速度
    std::vector<double>  target_tcp_acc;         // 目标末端加速度
};

// 实时数据
struct RealTimeData
{
    std::vector<double>  joint_pos_cmd;           // 关节位置实时指令, 仅在实时控制模式为关节位置时生效
    std::vector<double>  joint_vel_cmd;           // 关节速度实时指令, 仅在实时控制模式为关节速度时生效
    std::vector<double>  joint_torq_cmd;          // 关节力矩控制指令, 预留
    std::vector<double>  cart_pos_tool_wobj_cmd;  // 笛卡尔位置实时指令, 对应当前机器人工具坐标系在工件坐标系下的位置, 仅在实时控制模式未笛卡尔位置时生效
    std::vector<double>  cart_vel_tool_wobj_cmd;  // 笛卡尔速度实时指令, 对应当前机器人工具坐标系在工件坐标系下的速度, 仅在实时控制模式未笛卡尔速度时生效
    std::vector<double>  cart_ft_cmd;             // 笛卡尔力和力矩控制指令, 预留
    bool status;                                  // 控制指令刷新状态, 更新机器人控制指令时给true
};

class DucoCobot
{
public:
	/**
     * @brief 创建机器人远程连接的实例
     * @param ip 机器人IP
     * @param port 机器人端口(7003)
     */
    DucoCobot(std::string ip, unsigned int port);
	/**
     * @brief 连接机器人
     * @return -1:打开失败; 0:打开成功
     */
    int32_t open();
    /**
     * @brief 断开机器人连接
     * @return -1:打开失败; 0:打开成功
     */
    int32_t close();
    /**
     * @brief 机器人上电
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t power_on(bool block) ;
    /**
     * @brief 机器人下电
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t power_off(bool block) ;
    /**
     * @brief 机器人上使能
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t enable(bool block) ;
    /**
     * @brief 机器人下使能
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t disable(bool block) ;
    /**
     * @brief 机器人关机
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t shutdown(bool block) ;
    /**
     * @brief 停止所有任务
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t stop(bool block) ;
    /**
     * @brief 暂停所有任务
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t pause(bool block) ;
    /**
     * @brief 恢复所有暂停的任务
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t resume(bool block) ;
    /**
     * @brief 设置工具末端相对于法兰面坐标系的偏移, 设置成功后,
     *        后续运动函数中的TCP设置为该TCP或者为空时,使用该TCP参数
     * @param name           工具坐标系的名字
     * @param tool_offset    工具TCP偏移量 [x_off, y_off,z_off,rx,ry,rz], 单位: m, rad
     * @param payload        末端负载质量, 质心, [mass,x_cog,y_cog,z_cog], 单位: kg, m
     * @param inertia_tensor 末端工具惯量矩阵参数, 参数1-6分别对应矩阵xx、xy、xz、yy、yz、zz元素, 单位: kg*m^2
     * @return 返回当前任务结束时的状态
     */
    int32_t set_tool_data(const std::string& name, const std::vector<double> & tool_offset, const std::vector<double> & payload, const std::vector<double>& inertia_tensor) ;
    /**
     * @brief 设置工具末端相对于法兰面坐标系的偏移及质量、质心, 设置成功后, 后续运动函数中的TCP设置为该TCP或者为空时,使用该TCP参数,此函数会将工程文件中的值同步修改
     * @param name 工具坐标系的名称
     * @param tool 工具TCP偏移量 {x_off, y_off,z_off,rx,ry,rz}, 单位(m, rad)
     * @param payload 末端工具质量, 质心, {mass,x_cog,y_cog,z_cog}, 相对于法兰坐标系, 单位(kg, m)
     * @param inertia_tensor 末端工具惯量矩阵参数, 参数1-6分别对应矩阵xx、xy、xz、yy、yz、zz元素, 单位kg*m^2
     * @return 任务结束时的状态
     */
    int32_t set_tool_data_workcell(const std::string& name, const std::vector<double> & tool, const std::vector<double>& payload, const std::vector<double>& inertia_tensor) ;
    /**
     * @brief 获取当前状态下机械臂有效的末端工具的偏移量
     * @param _return TCP偏移量信息,单位: m, rad
     */
    void get_tcp_offset(std::vector<double> & _return) ;
    /**
     * @brief 设置工件坐标系
     * @param name 工件坐标系名称
     * @param wobj 工件坐标系
     * @return 返回当前任务结束时的状态
     */
    int32_t set_wobj(const std::string& name, const std::vector<double> & wobj) ;
    /**
     * @brief 根据名称修改要配置的工件坐标系, 此函数会将工程文件中的值同步修改
     * @param name 工件坐标系的名称
     * @param wobj {x, y, z, rx, ry, rz},单位(m, rad)
     * @return 任务结束时的状态
     */
    int32_t set_wobj_workcell(const std::string& name, const std::vector<double> & wobj);
    /**
     * @brief 基于当前的工件坐标系设置一个偏移量, 后续的move类脚本的参考工件坐标系上都将添加这个偏移量
     * @param wobj 工件坐标系相对于基坐标系的位移, 单位: m, rad
     * @param active 是否启用
     * @return 返回当前任务结束时的状态
     */
    int32_t set_wobj_offset(const std::vector<double> & wobj, bool active) ;
    /**
     * @brief 计算机械臂的正运动学, 求解给定TCP在给定wobj下的值
     * @param _return         末端姿态列表[x,y,z,rx,ry,rz]
     * @param joints_position 需要计算正解的关节角, 单位: rad
     * @param tool            工具坐标系信息,tcp偏移量[x_off,y_off,z_off,rx,ry,rz], 单位:m, rad, 为空使用当前tcp值
     * @param wobj            工件坐标系相对于基坐标系的位移[x, y, z, rx, ry, rz], 单位:m, rad, 为空使用当前wobj
     */
    void cal_fkine(std::vector<double> & _return, const std::vector<double> & joints_position, const std::vector<double> & tool, const std::vector<double> & wobj) ;
    /**
     * @brief 计算运动学逆解, 在求解过程中, 会选取靠近当前机械臂关节位置的解
     * @param _return   关节位置列表[q1,q2,q3,q4,q5,q6]
     * @param p      需要计算的末端位姿在设置工件坐标系的值,包含当前有效的工具偏移量, 单位:m,rad
     * @param q_near 用于计算逆运动学的参考关节位置,为空使用当前关节值
     * @param tool   工具坐标系信息,tcp偏移量[x_off,y_off,z_off,rx,ry,rz], 单位:m, rad, 为空使用当前tcp值
     * @param wobj   工件坐标系相对于基坐标系的位移[x, y, z, rx, ry, rz], 单位:m, rad, 为空使用当前wobj
     */
    void cal_ikine(std::vector<double> & _return, const std::vector<double> & p, const std::vector<double> & q_near, const std::vector<double> & tool, const std::vector<double> & wobj) ;
    /**
     * @brief set_tool_digital_out
     * @param num   机械臂末端的IO输出口序号, 范围从1-2
     * @param value true为高电平, false为低电平
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t set_tool_digital_out(int16_t num, bool value, bool block) ;
    /**
     * @brief 读取机械臂末端的IO输入口的高低电平, 返回true为高电平, false为低电平
     * @param num 机械臂末端的IO输出口序号, 范围从1-2
     * @return true为高电平, false为低电平
     */
    bool get_tool_digital_in(int16_t num) ;
    /**
     * @brief 读取机械臂末端的IO输入口的高低电平, 返回true为高电平, false为低电平
     * @param num 机械臂末端的IO输出口序号, 范围从1-2
     * @return true为高电平, false为低电平
     */
    bool get_tool_digital_out(int16_t num) ;
    /**
     * @brief 末端485端口读取长度为len的字节数据
     * @param data 读取到的数据, 未读到数据返回空列表
     * @param len  需要读取的长度
     */
    void tool_read_raw_data_485(std::vector<int8_t> & _return, int32_t len) ;
    /**
     * @brief 末端485匹配头head后读取到长度为len的一帧数据
     * @param data 读取到的数据, 未读到数据返回空列表
     * @param head 需要匹配的头数据
     * @param len  需要读取的长度
     */
    void tool_read_raw_data_485_h(std::vector<int8_t> & _return, const std::vector<int8_t> & head, int32_t len) ;
    /**
     * @brief 末端485匹配头head和尾tail读取到一帧匹配的数据
     * @param data 读取到的数据, 未读到数据返回空列表
     * @param head 需要匹配的头数据
     * @param tail 需要匹配的尾数据
     */
    void tool_read_raw_data_485_ht(std::vector<int8_t> & _return, const std::vector<int8_t> & head, const std::vector<int8_t> & tail) ;
    /**
     * @brief 末端485写原生数据, 将data写入485端口
     * @param data 写入的数据列表
     * @return true:成功, false:失败
     */
    bool tool_write_raw_data_485(const std::vector<int8_t> & data) ;
    /**
     * @brief 末端485写原生数据, 将data中的数据加上head写入485端口
     * @param data 写入的数据列表
     * @param head 添加的头
     * @return true:成功, false:失败
     */
    bool tool_write_raw_data_485_h(const std::vector<int8_t> & data, const std::vector<int8_t> & head) ;
    /**
     * @brief 末端485写原生数据, 将value中的数据加上头head和尾tail写入485端口
     * @param data 写入的数据列表
     * @param head 添加的头
     * @param tail 添加的尾
     * @return true:成功, false:失败
     */
    bool tool_write_raw_data_485_ht(const std::vector<int8_t> & data, const std::vector<int8_t> & head, const std::vector<int8_t> & tail) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到目标关节角状态
     * @param joints_list 1-6关节的目标关节角度, 单位: rad
     * @param v 关节角速度, 单位: 系统设定速度的百分比%, 取值范围(0,100]
     * @param a 关节角加速度, 单位: 系统设定加速度的百分比%, 取值范围(0,100]
     * @param r 融合半径, 单位: 系统设定最大融合半径的百分比%, 默认值为 0, 表示无融合, 取值范围[0,50)
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movej(const std::vector<double> & joints_list, double v, double a, double r, bool block, bool def_acc = false) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到目标关节角状态
     * @param joints_list 1-6关节的目标关节角度, 单位: rad
     * @param v 关节角速度, 范围[0.01*PI/180, 1.25*PI], 单位: rad/s
     * @param a 关节角加速度, 范围[0.01*PI/180, 12.5*PI], 单位: rad/s^2
     * @param r 融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movej2(const std::vector<double> & joints_list, double v, double a, double r, bool block, bool def_acc = false) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到末端目标位置
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 单位: rad
     * @param v 关节角速度, 单位: 系统设定速度的百分比%, 取值范围(0,100]
     * @param a 关节加速度, 单位: 系统设定加速度的百分比%, 取值范围(0,100]
     * @param r 融合半径, 单位: 系统设定最大融合半径的百分比%, 默认值为 0, 表示无融合, 取值范围[0,50)
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movej_pose(const std::vector<double> & p, double v, double a, double r, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc = false) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到末端目标位置
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 单位: rad
     * @param v 关节角速度, 范围[0.01*PI/180, 1.25*PI], 单位: rad/s
     * @param a 关节加速度, 范围[0.01*PI/180, 12.5*PI], 单位: rad/s^2
     * @param r 融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movej_pose2(const std::vector<double> & p, double v, double a, double r, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc = false) ;
    /**
     * @brief 控制机械臂末端从当前状态按照直线路径移动到目标状态
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 单位: rad
     * @param v 末端速度, 范围[0.00001, 5], 单位: m/s
     * @param a 末端加速度, 范围[0.00001, ∞), 单位: m/s^2
     * @param r 融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movel(const std::vector<double> & p, double v, double a, double r, const std::vector<double> & q_near, const std::string& tool = "default", const std::string& wobj = "default", bool block = true, bool def_acc = false) ;
    /**
     * @brief 控制机械臂做圆弧运动, 起始点为当前位姿点, 途径p1点, 终点为p2点
     * @param p1 圆弧运动中间点
     * @param p2 圆弧运动结束点
     * @param v  末端速度, 范围[0.00001, 5], 单位: m/s
     * @param a  末端加速度, 范围[0.00001, ∞), 单位: m/s^2
     * @param r  融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param mode   姿态控制模式  0:姿态与终点保持一致;1:姿态与起点保持一致;2:姿态受圆心约束
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movec(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double r, int mode, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc = false) ;
    /**
     * @brief 控制机械臂做圆周运动, 起始点为当前位姿点, 途径p1点和p2点
     * @param p1 圆周运动经过点
     * @param p2 圆周运动经过点
     * @param v  末端速度, 范围[0.00001, 5], 单位: m/s
     * @param a  末端加速度, 范围[0.00001, ∞), 单位: m/s^2
     * @param r  融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param mode   姿态控制模式   1:姿态与终点保持一致;  2:姿态受圆心约束
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t move_circle(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double r, int32_t mode, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, bool def_acc = false) ;
    /**
     * @brief 控制机械臂沿工具坐标系直线移动一个增量
     * @param pose_offset 工具坐标系下的位姿偏移量
     * @param v 直线移动的速度, 范围[0.00001, 5], 单位: m/s, 当x、y、z均为0时, 线速度按比例换算成角速度
     * @param a 加速度, 范围[0.00001, ∞), 单位: m/s^2
     * @param r 融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param tool   设置使用的工具坐标系的名称, 为空时默认为当前使用的工具坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t tcp_move(const std::vector<double> & pose_offset, double v, double a, double r, const std::string& tool, bool block, bool def_acc = false) ;
    /**
     * @brief 控制机器人沿工具坐标系直线移动一个增量, 增量为p1与p2点之间的差, 运动的目标点为:当前点*p1^-1*p2
     * @param p1 工具坐标系下的位姿偏移量计算点1
     * @param p2 工具坐标系下的位姿偏移量计算点2
     * @param v  直线移动的速度, 范围[0.00001, 5], 单位: m/s, 当x、y、z均为0时, 线速度按比例换算成角速度
     * @param a  加速度, 范围[0.00001, ∞), 单位: m/s^2
     * @param r  融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param tool  设置使用的工具坐标系的名称, 为空时默认为当前使用的工具坐标系
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t tcp_move_2p(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double r, const std::string& tool, const std::string& wobj, bool block, bool def_acc = false) ;
    /**
     * @brief 控制机械臂沿工件坐标系直线移动一个增量
     * @param pose_offset 工件坐标系下的位姿偏移量
     * @param v 直线移动的速度, 范围[0.00001, 5], 单位: m/s, 当x、y、z均为0时, 线速度按比例换算成角速度
     * @param a 加速度, 范围[0.00001, ∞), 单位: m/s^2
     * @param r 融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t wobj_move(const std::vector<double> & pose_offset, double v, double a, double r, const std::string& wobj, bool block, bool def_acc = false);
    /**
     * @brief 控制机器人沿工件坐标系直线移动一个增量, 增量为p1与p2点之间的差, 运动的目标点为:当前点*p1^-1*p2
     * @param p1 工件坐标系下的位姿偏移量计算点1
     * @param p2 工件坐标系下的位姿偏移量计算点2
     * @param v  直线移动的速度, 范围[0.00001, 5], 单位: m/s, 当x、y、z均为0时, 线速度按比例换算成角速度
     * @param a  加速度, 范围[0.00001, ∞), 单位: m/s^2
     * @param r  融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param tool  设置使用的工具坐标系的名称, 为空时默认为当前使用的工具坐标系
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t wobj_move_2p(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double r, const std::string& tool, const std::string& wobj, bool block, bool def_acc = false);
    /**
     * @brief 控制机械臂每个关节按照给定的速度一直运动, 函数执行后会直接运行后续指令.
     *        运行speedj函数后, 机械臂会持续运动并忽略后续运动指令, 直到接收到speed_stop()函数后停止
     * @param joints_list 每个关节的速度, 范围[0.01*PI/180, 1.25*PI], 单位: rad/s
     * @param a 主导轴的关节加速度, 范围[0.01*PI/180, 12.5*PI], 单位: rad/s^2
     * @param time  运行时间, 到达时间后会停止运动,单位: ms.默认-1表示一直运行
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t speedj(const std::vector<double> & joints_list, double a, int32_t time, bool block) ;
    /**
     * @brief 控制机械臂末端按照给定的速度一直运动, 函数执行后会直接运行后续指令.
     *        运行speedl函数后, 机械臂会持续运动并忽略后续运动指令, 直到接收到speed_stop()函数后停止
     * @param pose_list 末端速度向量, 范围[0.00001, 5], 线速度单位: m/s,角速度单位: rad/s
     * @param a 末端的线性加速度, 范围[0.00001, ∞), 单位: m/s^2
     * @param time  运行时间, 到达时间会停止运动, 单位: ms.默认-1表示一直运行
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t speedl(const std::vector<double> & pose_list, double a, int32_t time, bool block) ;
    /**
     * @brief 停止speedj及speedl函数的运动
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t speed_stop(bool block) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到目标关节角状态,运动过程中不考虑笛卡尔空间路径
     * @param joints_list 目标关节角度, 单位: rad
     * @param v 最大关节角速度, 范围[0.01*PI/180, 1.25*PI], 单位: rad/s
     * @param a 最大关节加速度, 范围[0.01*PI/180, 12.5*PI], 单位: rad/s^2
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 可缺省, 默认为非阻塞
     * @param kp 比例参数, 默认值200, 可缺省, 建议使用默认参数
     * @param kd 微分参数, 默认值25, 可缺省, 建议使用默认参数
     * @param smooth_vel 速度平滑参数, 默认值10, 可缺省, 范围[1-10], 当快速连续发送密集离散点位且无法保证点位间隔均匀性时, 推荐使用较大参数已保证速度的平滑度, 否则可根据实际需要降低参数提高精度
     * @param smooth_acc 加速度平滑参数, 默认值10, 可缺省, 范围[0-1], 当快速连续发送密集离散点位且无法保证点位间隔均匀性时, 推荐使用较大参数以保证加速度的平滑度, 否则可根据实际需要降低参数提高跟踪精度
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t servoj(const std::vector<double> & joints_list, double v, double a, bool block=false, double kp=200, double kd=25, double smooth_vel=10, double smooth_acc=1) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到目标笛卡尔状态,通过关节空间运动
     * @param pose_list 目标工件坐标系下的末端位姿, 单位: m, rad
     * @param v 关节速度, 范围[0.01*PI/180, 1.25*PI], 单位: rad/s
     * @param a 关节加速度, 范围[0.01*PI/180, 12.5*PI], 单位: rad/s^2
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 可缺省, 默认为非阻塞
     * @param kp 比例参数, 默认值200, 可缺省, 建议使用默认参数
     * @param kd 微分参数, 默认值25, 可缺省, 建议使用默认参数
     * @param smooth_vel 速度平滑参数, 默认值10, 可缺省, 范围[1-10], 当快速连续发送密集离散点位且无法保证点位间隔均匀性时, 推荐使用较大参数已保证速度的平滑度, 否则可根据实际需要降低参数提高精度
     * @param smooth_acc 加速度平滑参数, 默认值10, 可缺省, 范围[0-1], 当快速连续发送密集离散点位且无法保证点位间隔均匀性时, 推荐使用较大参数以保证加速度的平滑度, 否则可根据实际需要降低参数提高跟踪精度
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t servoj_pose(const std::vector<double> & pose_list, double v, double a, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block=false, double kp=200, double kd=25, double smooth_vel=10, double smooth_acc=1) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动一个增量,通过关节空间运动
     * @param pose_offset 目标工件坐标系下的末端位姿, 单位: m, rad
     * @param v 关节速度, 范围[0.01*PI/180, 1.25*PI], 单位: m/s
     * @param a 关节加速度, 范围[0.01*PI/180, 12.5*PI], 单位: m/s^2
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 默认值false, 可缺省
     * @param kp 比例参数, 默认值200, 可缺省, 建议使用默认参数
     * @param kd 微分参数, 默认值25, 可缺省, 建议使用默认参数
     * @param smooth_vel 速度平滑参数, 默认值10, 可缺省, 范围[1-10], 当快速连续发送密集离散点位且无法保证点位间隔均匀性时, 推荐使用较大参数已保证速度的平滑度, 否则可根据实际需要降低参数提高精度
     * @param smooth_acc 加速度平滑参数, 默认值10, 可缺省, 范围[0-1], 当快速连续发送密集离散点位且无法保证点位间隔均匀性时, 推荐使用较大参数以保证加速度的平滑度, 否则可根据实际需要降低参数提高跟踪精度
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t servo_tcp(const std::vector<double> & pose_offset, double v, double a, const std::string& tool, bool block=false, double kp=200, double kd=25, double smooth_vel=10, double smooth_acc=1) ;
    /**
     * @brief 控制机械臂末端从当前状态按照直线路径移动到目标状态
     * @param pose_list 目标工件坐标系下的末端位姿, 单位: m, rad
     * @param v 末端速度, 范围[0.00001, 5], 单位: m/s
     * @param a 末端加速度, 范围[0.00001, ∞), 单位: m/s^2
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 默认值false, 可缺省
     * @param kp 比例参数, 默认值200, 可缺省, 建议使用默认参数
     * @param kd 微分参数, 默认值25, 可缺省, 建议使用默认参数
     * @param smooth_vel 速度平滑参数, 默认值10, 可缺省, 范围[1-10], 当快速连续发送密集离散点位且无法保证点位间隔均匀性时, 推荐使用较大参数已保证速度的平滑度, 否则可根据实际需要降低参数提高精度
     * @param smooth_acc 加速度平滑参数, 默认值10, 可缺省, 范围[0-1], 当快速连续发送密集离散点位且无法保证点位间隔均匀性时, 推荐使用较大参数以保证加速度的平滑度, 否则可根据实际需要降低参数提高跟踪精度
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t servol(const std::vector<double> & pose_list, double v, double a, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block=false, double kp=200, double kd=25, double smooth_vel=10, double smooth_acc=1) ;
    /**
     * @brief 读取机器人最新的错误列表
     * @param _return 错误列表
     */
    void get_last_error(std::vector<std::string> & _return) ;
    /**
     * @brief 根据id查询当前的任务状态
     * @param id 任务的id
     * @return 任务的当前执行状态
     */
    int32_t get_noneblock_taskstate(int32_t id) ;
    /**
     * @brief 插入log日志, 记录运行问题
     * @param message 日志描述
     */
    void log_info(const std::string& message) ;
    /**
     * @brief 在运行过程中产生弹窗, 并暂停当前所有任务
     * @param message 弹窗描述
     */
    void log_error(const std::string& message) ;
    /**
     * @brief 切换机器人到仿真或者真机模式
     * @param sim   true:仿真, false:真机
     * @param block
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t simulation(bool sim, bool block) ;
    /**
     * @brief 设置机器人全局速度
     * @param val 机器人全局速度, 范围[1,100]
     * @return 任务结束时状态
     */
    int32_t speed(double val) ;
    /**
     * @brief 获取当前机器人状态
     * @param _return 机器人状态信息列表, data[0]表示机器人状态, data [1]表示程序状态,
     *             data [2]表示安全控制器状态, data [3]表示操作模式
     */
    void get_robot_state(std::vector<int8_t> & _return) ;
    /**
     * @brief 获取当前状态下机械臂末端法兰在基坐标系下的位姿
     * @param _return 末端法兰位置
     */
    void get_flange_pose(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂末端法兰在基坐标系下的速度
     * @param _return 末端法兰速度列表, 单位: m/s,rad/s
     */
    void get_flange_speed(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂末端法兰在基坐标系下的加速度
     * @param _return 末端法兰加速度列表, 单位: m/s^2, rad/s^2
     */
    void get_flange_acceleration(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂工具末端点在基坐标系下的位姿
     * @param _return 末端位姿
     */
    void get_tcp_pose(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂工具末端点的速度
     * @param _return 末端速度列表, 单位: m/s,rad/s
     */
    void get_tcp_speed(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂工具末端点的加速度
     * @param _return 末端加速度列表, 单位: m/s^2, rad/s^2
     */
    void get_tcp_acceleration(std::vector<double> & _return) ;
    /**
     * @brief 获取当前末端的力矩信息
     * @param _return 末端力矩信息, [Fx,Fy,Fz,Mx,My,Mz],单位: N、N.m
     */
    void get_tcp_force(std::vector<double> & _return) ;
    /**
     * @brief 获取机械臂工具末端在工具坐标系下的期望力矩信息
     * @param _return 末端力矩信息, [Fx,Fy,Fz,Mx,My,Mz],单位: N、N.m
     * @param tool 工具坐标系名称
     */
    void get_tcp_force_tool(std::vector<double> & _return, const std::string& tool);
    /**
     * @brief 获取当前状态下机械臂各关节的角度
     * @param _return 1-6轴关节角度列表, 单位: rad
     */
    void get_actual_joints_position(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节的规划角度
     * @param _return 1-6轴目标关节角度列表, 单位: rad
     */
    void get_target_joints_position(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节角速度
     * @param _return 1-6轴关节速度列表, 单位: rad/s
     */
    void get_actual_joints_speed(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节规划角速度
     * @param _return 1-6轴目标关节速度列表, 单位: rad/s
     */
    void get_target_joints_speed(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节角加速度
     * @param _return 1-6轴关节加速度列表, 单位: rad/s^2
     */
    void get_actual_joints_acceleration(std::vector<double> & _return) ;
    /**
     * @brief 取当前状态下机械臂各关节角规划加速度
     * @param _return 1-6轴关节加速度列表, 单位: rad/s^2
     */
    void get_target_joints_acceleration(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节力矩
     * @param _return 1-6轴关节力矩列表, 单位: N.m
     */
    void get_actual_joints_torque(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节目标力矩
     * @param _return 1-6轴关节加速度列表, 单位: rad/s^2
     */
    void get_target_joints_torque(std::vector<double> & _return) ;
        /**
     * @brief 设置碰撞检测等级
     * @param value 0:关闭碰撞检测, 1-5:对应设置碰撞检测等级1到等级5
     * @return 任务结束时状态
     */
    int32_t collision_detect(int32_t value) ;
    /**
     * @brief 设置抓取负载.可以在程序运行过程中设置机器人当前的负载(质量、质心)
     * @param value 末端工具抓取负载质量, 质心, {mass,x_cog,y_cog,z_cog},
     *              相对于工具坐标系, 质量范围[0, 35], 单位: kg, m
     * @return 任务结束时状态
     */
    int32_t set_load_data(const std::vector<double> & value) ;
    /**
     * @brief 控制机械臂开启末端力控.开启末端力控后所有运动函数除正常运动外,
     *        会额外基于已配置的末端力控参数进行末端力控运动
     * @param mode 力控模式选择,0为笛卡尔末端导纳,1为关节阻抗
     * @return 返回值代表当前任务的id信息
     */
    int32_t fc_start(int32_t mode) ;
    /**
     * @brief 控制机械臂退出末端力控
     * @return 当前任务的id信息
     */
    int32_t fc_stop() ;
    /**
     * @brief 修改并配置机器人末端力控参数
     * @param direction 6个笛卡尔空间方向末端力控开关, 开为true, 关为false
     * @param ref_ft 6个笛卡尔空间方向末端力控参考力, 范围[-1000, 1000], X/Y/Z方向单位: N,
                     RX/RY/RZ方向单位: Nm, 方向符号参考末端力控参考坐标系方向
     * @param mass  6个笛卡尔空间方向末端力控质量, 范围[0, 正无穷],
     *              X/Y/Z方向单位: kg, RX/RY/RZ方向单位: kg*m^2.
     * @param stiff  6个笛卡尔空间方向末端力控刚度, 范围[-1000, 1000],
     *              X/Y/Z方向单位: N/(m), RX/RY/RZ方向单位: Nm/(rad).
     * @param damp  6个笛卡尔空间方向末端力控阻尼, 范围[-10000, 10000],
     *              X/Y/Z方向单位: N/(m/s), RX/RY/RZ方向单位: Nm/(rad/s)
     * @param max_vel 6个笛卡尔空间方向末端力控最大调整速度, 范围[-5, 5], X/Y/Z方向单位: m/s,
     *                范围[-2*PI, 2*PI], RX/RY/RZ方向单位: rad/s
     * @param dead_zone 6个笛卡尔空间方向末端与环境接触力死区, 范围[-1000, 1000],
     *                     X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
     * @param toolname 设置使用的末端力控工具的名称, 默认为当前使用的工具
     * @param wobjname 设置使用的末端力控工件坐标系的名称, 默认为当前使用的工件坐标系
     * @param type 末端力控参考坐标系选择标志位, 0为参考工具坐标系, 1位参考工件坐标系
     * @param mode
     * @return 当前任务的id信息
     */
    int32_t fc_config(const std::vector<bool> & direction, const std::vector<double> & ref_ft, const std::vector<double> & mass,
                      const std::vector<double> & stiff, const std::vector<double> & damp, const std::vector<double> & max_vel, const std::vector<double> & dead_zone,
                      const std::string& toolname, const std::string& wobjname, const int32_t type, const int32_t mode) ;
    /**
     * @brief 控制机械臂仅产生末端力控运动, 为阻塞型指令
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 默认为false
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t fc_move(bool block = false) ;
    /**
     * @brief 控制机械臂在末端力控过程中进行力安全监控
     * @param direction 6个笛卡尔空间方向末端力安全监控开关, 开为true, 关为false
     * @param ref_ft 6个笛卡尔空间方向末端力安全监控参考力, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm,
     *               方向符号参考末端力安全监控参考坐标系方向
     * @param toolname 设置使用的末端力安全监控工具的名称
     * @param wobjname 设置使用的末端力安全监控工件坐标系的名称
     * @param type 末端力安全监控参考坐标系选择标志位, 0为参考工具坐标系, 1位参考工件坐标系
     * @param force_property 监控力属性, 0为末端负载力及外力, 1为末端外力(不含负载),可缺省, 默认为0
     * @return 当前任务的id信息
     */
    int32_t fc_guard_act(const std::vector<bool> & direction, const std::vector<double> & ref_ft, const std::string& toolname, const std::string& wobjname, int32_t type, int32_t force_property = 0) ;
    /**
     * @brief 控制机械臂在末端力控过程中禁用力安全监控
     * @return 当前任务的id信息
     */
    int32_t fc_guard_deact() ;
    /**
     * @brief 控制机械臂末端力传感器读数设置为指定值
     * @param direction 6个末端力传感器输出力设置标志位, 需要设置为true, 不需要设置为false
     * @param ref_ft 6个末端力传感器输出力设置目标值, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
     * @return 当前任务的id信息
     */
    int32_t fc_force_set_value(const std::vector<bool> & direction, const std::vector<double> & ref_ft) ;
    /**
     * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中满足指定位置判断条件时自动停止当前运动函数并调过后续运动函数,
     *        直到fc_stop()函数被执行停止末端力控
     * @param middle 位置判断条件绝对值, X/Y/Z方向单位: m, RX/RY/RZ方向单位: rad
     * @param range  位置判断条件偏移范围大小, X/Y/Z方向单位: m, RX/RY/RZ方向单位: rad
     * @param absolute 绝对/增量条件判断标志位, true为绝对位置判断, false为增量位置判断
     * @param duration 条件满足触发保持时间, 单位: ms
     * @param timeout  条件满足触发超时时间, 单位: ms
     * @return 当前任务的id信息
     */
    int32_t fc_wait_pos(const std::vector<double> & middle, const std::vector<double> & range, bool absolute, int32_t duration, int32_t timeout) ;
    /**
     * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中满足指定速度判断条件时自动停止当前运动函数并跳过后续运动函数,
     *        直到fc_stop()函数被执行停止末端力控
     * @param middle 速度判断条件绝对值, X/Y/Z方向范围[-5, 5], 单位: m/s, RX/RY/RZ方向范围[-2*PI, 2*PI], 单位: rad/s
     * @param range  速度判断条件偏移范围大小, X/Y/Z方向单位: m/s, RX/RY/RZ方向单位: rad/s
     * @param absolute 绝对/增量条件判断标志位, true为绝对速度判断, false为增量速度判断
     * @param duration 条件满足触发保持时间, 单位: ms
     * @param timeout  条件满足触发超时时间, 单位: ms
     * @return 当前任务的id信息
     */
    int32_t fc_wait_vel(const std::vector<double> & middle, const std::vector<double> & range, bool absolute, int32_t duration, int32_t timeout) ;
    /**
     * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中满足指定力判断条件时自动停止当前运动函数并跳过后续运动函数,
     *        直到fc_stop()函数被执行停止末端力控
     * @param middle 力判断条件绝对值, 范围[-1000, 1000], X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
     * @param range  力判断条件偏移范围大小, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
     * @param absolute 绝对/增量条件判断标志位, true为绝对力判断, false为增量力判断
     * @param duration 条件满足触发保持时间, 单位: ms
     * @param timeout  条件满足触发超时时间, 单位: ms
     * @return 当前任务的id信息
     */
    int32_t fc_wait_ft(const std::vector<double> & middle, const std::vector<double> & range, bool absolute, int32_t duration, int32_t timeout) ;
    /**
     * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中位置条件判断、速度条件判断与力条件判断间的逻辑关系.不配置时默认三个条件判断都禁用
     * @param value 三维整形列表, 0代表不启用, 1代表与逻辑, 2代表或逻辑.例如开启位置条件判断, 禁用速度条件判断, 开启力条件判断, 并且位置与力的关系为或, 则输入[1,0,2]
     * @return 当前任务的id信息
     */
    int32_t fc_wait_logic(const std::vector<int32_t> & value) ;
    /**
     * @brief 获取当前机器人末端传感器的反馈读数
     * @param _return 6自由度末端力读数, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
     */
    void fc_get_ft(std::vector<double> & _return) ;
    /**
     * @brief 获取当前机器人末端力控功能启用状态
     * @return 机器人末端力控启用返回true, 未启用返回false
     */
    bool fc_mode_is_active() ;
    /**
     * @brief 将一组points点位信息输入到机器人控制器中的轨迹池
     * @param track 一组points点位信息.每个point以6个double类型数据构成
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t trackEnqueue(const std::vector<std::vector<double> > & track, bool block);
    /**
     * @brief 将机器人控制器中的轨迹池清空
     * @return 当前任务结束时的状态
     */
    int32_t trackClearQueue();
    /**
     * @brief 获取机器人控制器中的当前轨迹池大小
     * @return 当前轨迹池大小
     */
    int32_t getQueueSize();
    /**
     * @brief 执行时, 机器人的各关节将顺序到达轨迹池中的点位值直到轨迹池中无新的点位.
     *        执行过程中, 主导关节(关节位置变化最大的关节)将以speed与acc规划运动, 其他关节按比例缩放.
     *        注:如果已经开始执行停止规划, 将不再重新获取轨迹池中的数据, 直到完成停止.
     *            停止后如果轨迹池中有新的点位, 将重新执行跟随.为保证运动连续性, 建议至少保证轨迹池中有10个数据
     * @param speed 最大关节速度, 范围[0.01*PI/180, 1.25*PI], 单位: rad/s
     * @param acc   最大关节加速度, 范围[0.01*PI/180, 12.5*PI], 单位: rad/s^2
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t trackJointMotion(double speed, double acc, bool block);
    /**
     * @brief 执行时, 机器人的工具末端tool将顺序到达轨迹池中的点位值直到轨迹池中无新的点位.
     *        执行过程中, 工具末端tool将以speed与acc在工件坐标系wobj下规划运动.
     *        注:如果已经开始执行停止规划, 将不再重新获取轨迹池中的数据, 直到完成停止.
                  停止后如果轨迹池中有新的点位, 将重新执行跟随.为保证运动连续性, 建议至少保证轨迹池中有10个数据
     * @param speed 最大末端速度, 范围[0.00001, 5], 单位: m/s
     * @param acc   最大末端加速度, 范围[0.00001, ∞), 单位: m/s^2
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @param tool  设置使用的工件坐标系的名称, 为空字符串时默认为当前使用的工件坐标系
     * @param wobj  设置使用的工具的名称,为空字符串时默认为当前使用的工具
     * @param radius 轨迹融合半径
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t trackCartMotion(double speed, double acc, bool block, const std::string& tool, const std::string& wobj, double radius);
    /**
     * @brief 确保机器人远程连接断开时, 机器人自动产生一条stop指令以停止当前运动.使用该函数需要单独创建一个线程周期性调用
     * @param time 心跳延时时间, 单位: ms
     */
    void rpc_heartbeat(int32_t time=1000);
    /**
     * @brief 获取当前RPC库的版本号
     * @return 当前RPC库的版本号
     */
    // 保留接口：客户端本地版本查询，不调用服务端RPC
    std::string get_version();
    /**
     * @brief 获取机器人当前的位姿等信息
     * @param status 机器人位姿等信息
     */
    void get_robot_status(RobotStatus& status);
    /**
     * @brief 获取末端法兰在工具坐标系和工件坐标系下的位姿
     * @param _return 末端法兰的位姿
     * @param tool 工具坐标系名称, 为空字符串默认为当前使用的坐标系
     * @param wobj 工件坐标系名称, 为空字符串默认为当前使用的坐标系
     */
    void get_tcp_pose_coord(std::vector<double> & _return, const std::string& tool, const std::string& wobj);
    /**
     * @brief 修改伺服参数
     * @param axis_num 关节索引号
     * @param id       参数的ID号
     * @param value    要设置的值
     * @param qfmt     要设置的qfmt值
     * @param block    指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t set_servo_config(int32_t axis_num, int32_t id, int32_t value,int32_t qfmt, bool block);
    /**
     * @brief 将伺服参数应用到实际控制
     * @param axis_num 关节索引号
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t apply_servo_config(int32_t axis_num, bool block);
    /**
     * @brief 获取电机极对数
     * @param _return 电机极对数
     */
    void get_motor_pole_pair_number(std::vector<int16_t> & _return);
    /**
     * @brief 获取机器人轴电机定子插槽编号
     * @param _return 轴电机定子插槽编号
     */
    void get_motor_stator_slots(std::vector<int16_t> & _return);
    /**
     * @brief 获取机器人轴减速器比率
     * @param _return 轴减速器比率
     */
    void get_axis_ratio(std::vector<int16_t> & _return);
    /**
     * @brief 重置碰撞检测警告
     * @return 任务结束时状态
     */
    int32_t collision_detection_reset();
    /**
     * @brief 修改底层伺服配置文件的参数
     * @param axis_num 关节索引号
     * @param id       参数的ID号
     * @param name     参数的名称
     * @param value    要设置的值
     * @param qfmt     要设置的qfmt值
     * @return 任务结束时状态
     */
    int32_t set_servo_file_params(int32_t axis_num, int32_t id, const std::string& name, double value, double qfmt);
    /**
     * @brief 控制机械臂在关节或者笛卡尔空间做点动
     * @param param Jog运动的相关参数, 参考MoveJogTaskParam
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t move_jog(const MoveJogTaskParam& param, bool block);
    /**
     * @brief 结束机械臂的关节或者笛卡尔Jog
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t stop_manual_move(bool block);
    /**
     * @brief 获取机器人控制器的软件版本号
     * @param _return 机器人控制器的软件版本号
     */
    void get_robot_version(std::string& _return);
    /**
     * @brief 获取全局速度的百分比
     * @return 全局速度的百分比
     */
    int32_t get_global_speed();
    /**
     * @brief 外部轴使能
     * @param name 机器人名称("waist","head")
     * @param axis_num 关节轴号(1-N)
     * @return 任务结束时的状态
     */
    int32_t enable_eaxis(const std::string& name, int32_t axis_num);
    /**
     * @brief 外部轴下使能
     * @param name 机器人名称("waist","head")
     * @param axis_num 关节轴号(1-N)
     * @return 任务结束时的状态
     */
    int32_t disable_eaxis(const std::string& name, int32_t axis_num);
    /**
     * @brief 控制外部轴移动
     * @param name 目标机器人名称("waist","head")
     * @param epose 目标机器人所对应自由度位置,
     *              记录位置自由度及单位根据轴所设置自由度及类型改变, 单位: rad, m
     * @param v     各自由度最大规划速度, 根据对应轴类型改变, 单位: rad/s, m/s
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t move_eaxis(const std::string& name, const std::vector<double> & epose, const std::vector<double> & v, const bool block);
    /**
     * @brief 控制外部轴移动
     * @param name 目标机器人名称(当前仅支持"waist")
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 单位: rad
     * @param v 末端速度, 单位: m/s
     * @param a 末端加速度, 单位: m/s^2
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movel_eaxis(const std::string& name, const std::vector<double> & p, const double v, const double a, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, const bool block);
    /**
     * @brief 控制外部轴从当前状态, 按照关节运动的方式移动到目标外部轴状态
     * @param name 目标机器人名称("waist","head")
     * @param epose 目标机器人所对应自由度位置, 根据对应轴类型改变, 单位: rad, m
     * @param v 各自由度最大规划速度, 根据对应轴类型改变, 单位: rad/s, m/s
     * @param a 各自由度最大规划加速度, 根据对应轴类型改变, 单位: rad/s^2, m/s^2
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 可缺省, 默认为非阻塞
     * @param kp 比例参数, 默认值200, 可缺省, 建议使用默认参数
     * @param kd 微分参数, 默认值25, 可缺省, 建议使用默认参数
     * @param smooth_vel 速度平滑参数, 默认值10, 可缺省, 范围[1-10]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数已保证速度的平滑度，否则可根据实际需要降低参数提高精度
     * @param smooth_acc 加速度平滑参数, 默认值1, 可缺省, 范围[0-1]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数以保证加速度的平滑度，否则可根据实际需要降低参数提高跟踪精度
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t servo_eaxis(const std::string& name, const std::vector<double> & epose, const std::vector<double> & v, const std::vector<double> & a, const bool block = false, double kp = 200, const double kd = 25, double smooth_vel = 10, double smooth_acc = 1);
    /**
     * @brief 控制外部轴和机器人联动, 且机器人从当前状态按照直线路径移动到目标状态
     * @param name 目标机器人名称(当前仅支持"waist")
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 单位: rad
     * @param v 末端速度, 单位: m/s
     * @param a 末端加速度, 单位: m/s^2
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空列表时使用当前位置
     * @param tool   设置使用的工具的名称, 为空字符串时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空字符串时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 可缺省, 默认为非阻塞
     * @param kp 比例参数, 默认值200, 可缺省, 建议使用默认参数
     * @param kd 微分参数, 默认值25, 可缺省, 建议使用默认参数
     * @param smooth_vel 速度平滑参数, 默认值10, 可缺省, 范围[1-10]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数已保证速度的平滑度，否则可根据实际需要降低参数提高精度
     * @param smooth_acc 加速度平滑参数, 默认值1, 可缺省, 范围[0-1]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数以保证加速度的平滑度，否则可根据实际需要降低参数提高跟踪精度
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t servol_eaxis(const std::string& name, const std::vector<double> & p, const double v, const double a, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj,
                         bool block = false, double kp = 200, const double kd = 25, double smooth_vel = 10, double smooth_acc = 1);
    /**
     * @brief 开启实时控制模式
     * @param mode  实时控制模式, 0：关节位置, 1：关节速度, 2：关节力矩, 3：空间位置, 4：空间速度
     * @param filter_bandwidth 实时控制指令滤波器带宽, 单位Hz, 默认100Hz
     * @param com_lost_time 实时控制通讯数据丢失监控保护时间, 单位s, 默认0.02s
     * @return 任务结束时的状态
     */
    int32_t start_realtime_mode(int32_t mode, double filter_bandwidth, double com_lost_time);
    /**
     * @brief 结束实时控制模式
     * @return 任务结束时的状态
     */
    int32_t end_realtime_mode();
    /**
     * @brief 实时数据入队
     * @param realtime_data 实时数据
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 任务结束时的状态
     */
    int32_t realtime_data_enqueue(const std::vector<RealTimeData> &realtime_data, bool block);
    /**
     * @brief 清空实时数据队列
     * @return 任务结束时的状态
     */
    int32_t clear_realtime_data_queue();
    /**
     * @brief 获取当前实时队列池数据的数量
     * @return 当前实时队列池数据的数量
     */
    int32_t get_realtime_data_queue_size();
    /**
     * @brief 手自动模式切换
     * @param mode 0:手动模式, 1:自动模式.
     * @return 阻塞执行代表任务结束时状态
    */
    int32_t switch_mode(const int32_t mode);
    /**
     * @brief 当前机器人型号原始DH参数
     * @param dh 原始DH参数, 参数顺序a, alpha, d, theta, 单位m/rad
     */
    void get_origin_DH(std::vector<std::vector<double> > & dh);
    /**
     * @brief 当前机器人型号标定补偿后DH参数
     * @param calib_dh 补偿后DH参数, 参数顺序a, alpha, d, theta, 单位m/rad
     */
    void get_calib_DH(std::vector<std::vector<double> > & calib_dh);
    /**
     * @brief 获取机器人系列号, 型号, ext, SN
     * @param type 机器人系列号, 型号, ext, SN
     */
    void get_robot_type(std::vector<std::string> & type);
    /**
     * @brief 获取关节外力矩
     * @param torque 关节外力矩
     */
    void get_ext_torque(std::vector<double> & torque);
    /**
     * @brief 修改SJxx-xx-x.json文件的Fric值和零位
     * @param params Fric值和零位
     * @return 任务结束时的状态
     */
    int32_t set_dynamic_calibration_params(const std::vector<std::vector<double> > & params);
    /**
     * @brief 获取SJxx-xx-x.json文件的Fric值和零位
     * @param params Fric值和零位
     * @return
     */
    void get_dynamic_calibration_params(std::vector<std::vector<double> > & params);
    /**
     * @brief set_dynamic_bias_params
     * @param active
     * @param bias_param
     * @return
     */
    int32_t set_dynamic_bias_params(const std::vector<bool> & active, const std::vector<double> & bias_param);
    /**
     * @brief set_dynamic_calibration_minparams
     * @param min_params
     * @param g
     * @return
     */
    int32_t set_dynamic_calibration_minparams(const std::vector<double> & min_params, const std::vector<double> & g);
    /**
     * @brief get_dynamic_calibration_minparams
     * @param _return
     */
    void get_dynamic_calibration_minparams(std::vector<double> & _return);
    /**
     * @brief get_dynamic_g
     * @param _return
     */
    void get_dynamic_g(std::vector<double> & _return);
    /**
     * @brief 同步机器人参数到末端
     * @param passwd 密码
     * @return 任务结束时的状态
     */
    int32_t upload_robot_param_to_toolboard(const std::string& passwd);
    /**
     * @brief get_joint_gravtorque
     * @param _return
     */
    void get_joint_gravtorque(std::vector<double> & _return);
    /**
     * @brief get_auxiliary_position
     * @param _return
     */
    void get_auxiliary_position(std::vector<double> & _return);
    /**
     * @brief get_joint_dynamic_torque
     * @param _return
     */
    void get_joint_dynamic_torque(std::vector<double> & _return);
    /**
     * @brief 获取碰撞状态
     * @return 碰撞状态
     */
    bool get_collision_state();
    /**
     * @brief 发生碰撞的关节
     * @return 碰撞的关节
     */
    int32_t get_collision_num();
    /**
     * @brief save_dynamic_calibration_minparams
     * @param passwd
     * @return
     */
    int32_t save_dynamic_calibration_minparams(const std::string& passwd);

    /**
     * @brief 获取机器人轴减速器比率
     * @param rated 轴减速器比率
     */
    void get_axis_motor_rated_current(std::vector<double> & rated);
    /**
     * @brief 获取机器人轴减速器比率
     * @param kt 轴减速器比率
     */
    void get_axis_motor_kt(std::vector<double> & kt);
    /**
     * @brief 修改机器人DH参数
     * @param params 机器人DH参数 a alpha d beta
     * @return 任务结束时的状态
     */
    int32_t set_kinematic_calibration_params(const std::vector<std::vector<double> > & params);
    /**
     * @brief set_stiffness_calibration_params
     * @param params
     * @return
     */
    int32_t set_stiffness_calibration_params(const std::vector<std::vector<double> > & params);
    /**
     * @brief get_stiffness_calibration_params
     * @param params
     */
    void get_stiffness_calibration_params(std::vector<std::vector<double> > & params);
    /**
     * @brief save_stiffness_calibration_params
     * @param passwd
     * @return
     */
    int32_t save_stiffness_calibration_params(const std::string& passwd);
    /**
     * @brief get_joint_motion_params
     * @param params
     */
    void get_joint_motion_params(std::vector<std::vector<double> > & params);
    /**
     * @brief 获取机器人零位信息
     * @param bias 零位信息
     */
    void get_pos_bias(std::vector<double> & bias);
    /**
     * @brief 获取机器人pose_list类型全局变量的值
     * @param value pose_list类型全局变量的值
     * @param name 全局变量的名称. 如果变量在域里, 名称格式为："域名.变量名"
     */
    void get_system_value_lists(std::vector<std::vector<double> > & value, const std::string& name);
    /**
     * @brief 设置运动学标定识别码
     * @param passwd 密码
     * @param version 版本号
     * @return 任务结束时的状态
     */
    int32_t set_kinematic_calibration_info(const std::string& passwd, const std::string& version);
    /**
     * @brief 设置动力学标定识别码
     * @param passwd 密码
     * @param version 版本号
     * @return 任务结束时的状态
     */
    int32_t set_dynamic_calibration_info(const std::string& passwd, const std::string& version);
    /**
     * @brief 设置震动标定识别码
     * @param passwd  密码
     * @param info 震动标定信息
     * @return 任务结束时的状态
     */
    int32_t set_vibration_calibration_info(const std::string& passwd, const std::string& info);
    /**
     * @brief 获取震动参数
     * @param params
     */
    void get_vibration_calibration_params(std::vector<std::vector<double> > & params);
    /**
     * @brief 保存运动学参数到文件
     * @param passwd 密码
     * @return 任务结束时的状态
     */
    int32_t save_kinematic_calibration_params(const std::string& passwd);
    /**
     * @brief 保存动力学参数到文件
     * @param passwd 密码
     * @return 任务结束时的状态
     */
    int32_t save_dynamic_calibration_params(const std::string& passwd);
    /**
     * @brief set_vibration_params
     * @param vibration_on
     * @param vibration_mode
     * @return
     */
    int32_t set_vibration_params(const bool vibration_on, const int32_t vibration_mode);
    /**
     * @brief get_vibration_on
     * @return
     */
    bool get_vibration_on();
    /**
     * @brief get_vibration_mode
     * @return
     */
    int32_t get_vibration_mode();
    /**
     * @brief get_eaxis_status
     * @param status
     * @param name
     */
    void get_eaxis_status(RobotStatus& status, const std::string& name);
    /**
     * @brief 获取当前设置工具的负载质量及质心位置
     * @param _return 质量单位: kg,质心位置单位: m,[mass,x_cog,y_cog,z_cog]
     */
    void get_tool_load(std::vector<double> & load);
    /**
     * @brief 获取当前设置的工件坐标系的值
     * @param wobj [x, y, z, rx, ry, rz]工件坐标系相对于基坐标系的位移, 单位: m, rad
     */
    void get_wobj(std::vector<double> & wobj);
    /**
     * @brief 设置机械臂的安装方式
     * @param installation 安装方式。 两个元素{ry, rz},ry:机器人绕基座倾斜弧度, rz:机器人绕基座旋转弧度
     * @return  返回值代表当前任务结束时的状态
     */
    int32_t set_installation(const std::vector<double> & installation);
    /**
     * @brief 配置力控控制模式
     * @param mode 力控模式选择, 0为笛卡尔末端导纳, 1为关节阻抗， 2为笛卡尔末端阻抗
     * @return 返回值代表当前任务的id信息
     */
    int32_t fc_set_mode(const int32_t mode);
    /**
     * @brief 修改并配置机器人关节阻抗力控参数
     * @param direction 7个关节空间方向力控开关, 开为true, 关为false
     * @param damp  7个关节空间方向力控阻尼, 范围[-10000, 10000],
     *              单位: Nm/(rad/s)
     * @param stiff  7个关节空间方向力控刚度, 范围[-1000, 1000],
     *              单位: Nm/(rad)
     * @return 返回值代表当前任务的id信息
     */
    int32_t fc_config_joint(const std::vector<bool> & direction, const std::vector<double> & stiff, const std::vector<double> & damp);
    /**
     * @brief 修改并配置机器人笛卡尔阻抗力控参数
     * @param direction 6个笛卡尔空间方向末端力控开关, 开为true, 关为false
     * @param ref_ft  6个笛卡尔空间方向末端力控参考力矩, 范围[-1000, 1000]
     *              单位: N | Nm
     * @param damp  6个笛卡尔空间方向末端力控阻尼, 位置阻尼[-2000, 2000], 姿态阻尼[-100, 100],
     *              单位: N/(m/s) | Nm/(rad/s)
     * @param stiff  6个笛卡尔空间方向末端力控刚度, 位置刚度[-15000, 15000], 姿态刚度[-2000, 2000]
     *              单位: N/(m) | Nm/(rad)
     * @param dead_zone 6个笛卡尔空间方向末端与环境接触力死区, 范围[-1000, 1000],
     *                     X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
     * @param toolname 设置使用的末端力控工具的名称, 默认为当前使用的工具
     * @param wobjname 设置使用的末端力控工件坐标系的名称, 默认为当前使用的工件坐标系
     * @param type 末端力控参考坐标系选择标志位, 0为参考工具坐标系, 1位参考工件坐标系
     * @return 返回值代表当前任务的id信息
     */
    int32_t fc_config_cart(const std::vector<bool> & direction, const std::vector<double> & ref_ft, const std::vector<double> & stiff,
                           const std::vector<double> & damp, const std::vector<double> & dead_zone, const std::string& tool, const std::string& wobj, const int32_t type);
    /**
     * @brief 控制机器人进入牵引示教模式
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t teach_mode(bool block) ;
    /**
     * @brief 控制机器人退出牵引示教模式
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t end_teach_mode(bool block) ;
    /**
     * @brief 开启servo运动优化相关功能
     * @param vel_limit 是否开启关节速度保护
     * @param pos_limit 是否开启关节位置保护
     * @param kine_limit 是否开启运动学求解保护
     * @return 任务结束时状态
     */
    int32_t enable_servo_optimization(bool vel_limit, bool pos_limit, bool kine_limit);
    /**
     * @brief 关闭servo运动优化相关功能
     * @return 任务结束时状态
     */
    int32_t disable_servo_optimization();

private:
    std::shared_ptr<apache::thrift::transport::TSocket> _socket;        // SDK内部使用, 开发人员无需关心
    std::shared_ptr<apache::thrift::transport::TTransport> _transport;  // SDK内部使用, 开发人员无需关心
    std::shared_ptr<apache::thrift::protocol::TProtocol> _protocol;     // SDK内部使用, 开发人员无需关心
    std::shared_ptr<RPCRobotClient> _robot;                             // SDK内部使用, 开发人员无需关心
    std::string _serverIP;                                              // 机器人IP
};
}
#endif // _DUCOCOBOT
