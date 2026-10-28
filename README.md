# DUCO 机器人 C++ SDK

[简体中文](README.md) | [English](README_EN.md)

本 SDK 通过 Thrift RPC 与机器人控制器通信，提供机械臂、腰部和头部的 C++ 接口，支持连接管理、状态查询和运动控制。机械臂接口还包含坐标系、数字 I/O、轨迹及实时控制等功能。

所有高层接口位于 `DucoAPI` 命名空间。使用前应确认控制器的服务端口、协议版本和实际关节数量。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| [src/DucoCobot.h](src/DucoCobot.h)、`src/DucoCobot.cpp` | 机械臂接口 |
| [src/DucoCobotWaist.h](src/DucoCobotWaist.h)、`src/DucoCobotWaist.cpp` | 腰部接口 |
| [src/DucoCobotHead.h](src/DucoCobotHead.h)、`src/DucoCobotHead.cpp` | 头部接口 |
| `src/thrift/` | 随 SDK 提供的 Thrift C++ 运行库 |
| `src/thrift/gen-cpp/` | 生成的 RPC 客户端与数据类型 |
| [examples/Move.cpp](examples/Move.cpp) | 左右臂顺序查询及可选运动示例 |
| [examples/Components.cpp](examples/Components.cpp) | 腰部、头部状态查询示例 |
| [scripts/build-mingw.ps1](scripts/build-mingw.ps1) | Windows MinGW 构建脚本 |

业务代码使用三个高层接口类即可。生成的 RPC 代码应与控制器协议配套，不建议直接修改。

## 环境要求

| 平台 | 编译环境 | 构建方式 |
| --- | --- | --- |
| Windows | MinGW-w64 GCC/G++，支持 C++11 和 `std::thread`，同一工具链的 `ar.exe` | PowerShell 构建脚本 |
| Linux | 支持 C++11 的 C/C++ 编译器、系统开发头文件及线程库 | CMake、Make 或 Ninja |

基础构建使用仓库内的 Thrift 源码，无需另外安装 Thrift 编译器、Python 或 Qt。Windows MinGW GCC 7.3 已完成编译验证；MSVC/Visual Studio 尚未适配验证。当前顶层 CMake 的 Windows 分支未包含完整 SDK，Windows 请使用 MinGW 脚本。

以下构建及运行命令均在 SDK 根目录执行。

## 编译 SDK

### Windows

将 `$mingwBin` 替换为实际 MinGW 工具链的 `bin` 目录：

```powershell
$mingwBin = 'C:\mingw64\bin'
$env:PATH = "$mingwBin;$env:PATH"

powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\build-mingw.ps1 -Compiler "$mingwBin\g++.exe"
```

`-ExecutionPolicy Bypass` 仅对本次 PowerShell 子进程有效。可追加 `-RunTests` 运行本机回环测试；`-BuildDirectory` 可指定输出目录。

默认产物位于 `build-mingw/`：

- `libDucoCobotAPI.a`：SDK 静态库。
- `duco_move.exe`：左右臂示例。
- `duco_components.exe`：头腰示例。
- `duco_component_rpc_test.exe`：回环测试程序。

运行程序时，需保证同一 MinGW 工具链的运行库 DLL 可从 `PATH` 找到。脚本当前使用 `-O0 -g` 构建。

### Linux

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DDUCO_BUILD_EXAMPLES=ON
cmake --build build --parallel
```

上述命令使用 CMake 3.13 及以上的命令形式。项目声明的最低版本为 3.5，旧版本可手动创建并进入 `build` 目录后执行 `cmake ..`、`cmake --build .`。

默认生成 `build/libDucoCobotAPI.so`，示例位于 `build/examples/`。`DUCO_BUILD_EXAMPLES` 默认关闭；仅编译 SDK 时可省略此选项。当前 Linux 整库构建尚未在本环境验证。

## 连接控制器

| 部件 | 接口类 | 示例端口 |
| --- | --- | --- |
| 左臂 | `DucoAPI::DucoCobot` | `7003` |
| 右臂 | `DucoAPI::DucoCobot` | `7004` |
| 腰部 | `DucoAPI::DucoCobotWaist` | `7020` |
| 头部 | `DucoAPI::DucoCobotHead` | `7021` |

IP 应填写实际控制器地址，并保证对应 TCP 端口可达。每个部件使用独立客户端，客户端类型必须与服务匹配。上述端口是示例配置，实际以控制器为准。

构造对象不会立即连接，需调用 `open()`。构造函数设置 3000 ms 连接超时；端口大于 `9000` 时使用 `TFramedTransport`，其余使用 `TBufferedTransport`，协议均为 `TBinaryProtocol`。

### 最小示例：连接并读取关节角

将以下代码保存为 `main.cpp`，并将 IP 改为实际控制器地址：

```cpp
#include "DucoCobot.h"
#include <exception>
#include <iostream>
#include <vector>

int main()
{
    DucoAPI::DucoCobot robot("192.168.120.225", 7003);
    if (robot.open() != 0) {
        std::cerr << "Connection failed\n";
        return 1;
    }

    int exit_code = 0;
    try {
        std::vector<double> joints;
        robot.get_actual_joints_position(joints);
        for (double q : joints) std::cout << q << ' ';
        std::cout << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        exit_code = 1;
    }

    if (robot.close() != 0) exit_code = 1;
    return exit_code;
}
```

Windows MinGW 编译此程序，使用前面生成的静态库：

```powershell
g++ -std=c++11 -Isrc main.cpp build-mingw/libDucoCobotAPI.a -pthread -lws2_32 -o robot_app.exe
.\robot_app.exe
```

Linux 编译并运行：

```bash
g++ -std=c++11 -Isrc main.cpp -Lbuild -lDucoCobotAPI -pthread -o robot_app
export LD_LIBRARY_PATH="$PWD/build:${LD_LIBRARY_PATH:-}"
./robot_app
```

应用与 SDK 必须使用兼容的编译工具链、运行库和目标架构。MinGW 生成的静态库不直接作为 MSVC 库使用。

## 基本调用约定

| 接口或参数 | 含义 |
| --- | --- |
| `open()`、`close()` | `0` 成功，`-1` 失败；连接失败后不要继续调用 RPC |
| 任务接口 `block=true` | 返回任务结束时的状态 |
| 任务接口 `block=false` | 返回任务 ID，通过 `get_noneblock_taskstate(id)` 查询 |
| 查询接口的引用参数 | 用于接收返回数据，如 `get_actual_joints_position(joints)` |
| `get_version()` | 获取客户端 SDK 版本 |
| `get_robot_version(version)` | 通过输出参数获取控制器版本 |

任务状态定义在 `DucoAPI::TaskState` 中。无融合运动正常结束通常返回 `ST_Finished`（值为 `4`），不能按连接接口的 `0` 成功约定判断。融合运动可能返回 `ST_Interrupt`，应按具体接口说明处理。除连接接口自行返回错误码外，RPC 通信异常通常向调用方抛出。

### 单位与参数

| 数据或接口 | 单位或说明 |
| --- | --- |
| 关节位置、速度、加速度 | rad、rad/s、rad/s²；列表长度与实际关节数一致 |
| 笛卡尔位姿 | `[x,y,z,rx,ry,rz]`，位置 m、姿态 rad |
| `movej`、`movej_pose` 的 `v/a` | 系统设定速度、加速度的百分比 |
| `movej2`、`movej_pose2` 的 `v/a` | rad/s、rad/s² |
| `movel` 的 `v/a` | m/s、m/s² |
| 融合参数 `r` | `0` 为不融合；不同接口的单位不同，见头文件 |
| 运动接口的 `tool/wobj` | 通常为坐标系名称，空字符串使用当前配置 |
| `cal_fkine/cal_ikine` 的 `tool/wobj` | 数值向量，空向量使用当前配置 |

机械臂使能接口为 `enable(block)`；腰部、头部为 `enable(axis_num, block)`，轴编号及取值由控制器定义。坐标系设置接口会影响后续运动，`set_tool_data_workcell`、`set_wobj_workcell` 还会同步修改工程文件。

### 连接与心跳

连接由主流程持有并跨业务步骤复用，在整个业务退出时统一关闭。不要为每次运动重新建立连接，也不要在多个线程中并发调用同一个客户端。

需要心跳时，为同一部件建立独立连接，在单独线程中周期调用 `rpc_heartbeat(time)`，参数单位为 ms，默认 `1000`。线程异常应由主流程检查并处理，退出时先回收线程，再关闭连接，具体写法见 `Move.cpp`。

连接超时不等于 RPC 收发超时。当前高层封装未暴露收发超时设置，网络异常时调用或线程回收可能等待响应。`close()` 本身不主动停止、下使能、下电或复位输出；心跳断连保护由控制器处理，应用需要停止时应显式调用对应接口。

## 运行示例

### 左右臂示例

在 [Move.cpp](examples/Move.cpp) 顶部直接配置：

```cpp
const string ROBOT_IP = "192.168.120.225";
const unsigned int LEFT_ARM_PORT = 7003;
const unsigned int RIGHT_ARM_PORT = 7004;
const bool RUN_MOTION = false;
```

重新编译后，无需传入端口或执行参数：

```powershell
.\build-mingw\duco_move.exe
```

Linux 对应命令为 `./build/examples/duco_move`。程序先完成左臂流程，再执行右臂流程；两侧连接在全部业务结束后统一关闭。默认只查询状态、正逆解和末端 I/O。

需要运动时，将 `RUN_MOTION` 改为 `true` 并重新编译。程序会为两侧维护独立心跳，依次使能、执行阻塞运动；每侧第一轴在当前位置增加 `0.02 rad`，速度为 `0.1 rad/s`、加速度为 `0.2 rad/s²`。运行前按现场流程上电并确认目标与运动空间，修改运动代码时核对实际关节数量。运动分支尚未试运行。

### 头腰示例

在 [Components.cpp](examples/Components.cpp) 顶部设置 `ROBOT_IP`、`WAIST_PORT` 和 `HEAD_PORT`，重新编译后运行：

```powershell
.\build-mingw\duco_components.exe
```

Linux 对应命令为 `./build/examples/duco_components`。该示例仅查询腰部和头部状态，不执行运动。两个示例均允许仅通过一个可选启动参数覆盖 IP。

## 更多说明

- 各接口的参数、默认值与返回值：见 `src/DucoCobot*.h` 中的中文注释。
- 本机回环测试：Windows 构建时追加 `-RunTests`；Linux 使用 `-DDUCO_BUILD_TESTS=ON` 构建后运行 CTest。
- Thrift 上游说明及许可证说明：[Thrift README](docs/thrift-README.md)。
