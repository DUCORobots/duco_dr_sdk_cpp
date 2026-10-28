# DUCO Robot C++ SDK

[简体中文](README.md) | [English](README_EN.md)

This SDK communicates with robot controllers through Thrift RPC. It provides C++ interfaces for the arms, waist, and head, including connection management, state queries, and motion control. The arm interface also supports coordinate systems, digital I/O, trajectories, and real-time control.

All high-level interfaces are in the `DucoAPI` namespace. Before use, confirm the controller's service ports, protocol version, and actual number of joints.

## Directory structure

| Path | Description |
| --- | --- |
| [src/DucoCobot.h](src/DucoCobot.h), `src/DucoCobot.cpp` | Arm interface |
| [src/DucoCobotWaist.h](src/DucoCobotWaist.h), `src/DucoCobotWaist.cpp` | Waist interface |
| [src/DucoCobotHead.h](src/DucoCobotHead.h), `src/DucoCobotHead.cpp` | Head interface |
| `src/thrift/` | Bundled Thrift C++ runtime |
| `src/thrift/gen-cpp/` | Generated RPC clients and data types |
| [examples/Move.cpp](examples/Move.cpp) | Sequential left/right arm queries and optional motion |
| [examples/Components.cpp](examples/Components.cpp) | Waist and head state queries |
| [scripts/build-mingw.ps1](scripts/build-mingw.ps1) | Windows MinGW build script |

Application code can use the three high-level client classes. Generated RPC code must match the controller protocol and should generally not be edited directly.

## Requirements

| Platform | Compiler environment | Build tools |
| --- | --- | --- |
| Windows | MinGW-w64 GCC/G++ with C++11 and `std::thread` support; `ar.exe` from the same toolchain | PowerShell build script |
| Linux | C/C++ compiler with C++11 support, system development headers, and thread library | CMake, Make or Ninja |

The basic build uses the bundled Thrift source. No separate Thrift compiler, Python, or Qt installation is required. Windows builds have been verified with MinGW GCC 7.3; MSVC/Visual Studio support has not been adapted or verified. The Windows branch of the top-level CMake configuration does not include the complete SDK, so use the MinGW script on Windows.

Run all build and execution commands below from the SDK root directory.

## Building the SDK

### Windows

Set `$mingwBin` to the actual `bin` directory of your MinGW toolchain:

```powershell
$mingwBin = 'C:\mingw64\bin'
$env:PATH = "$mingwBin;$env:PATH"

powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\build-mingw.ps1 -Compiler "$mingwBin\g++.exe"
```

`-ExecutionPolicy Bypass` applies only to this PowerShell child process. Add `-RunTests` to run the local loopback tests. Use `-BuildDirectory` to specify an output directory.

By default, the following files are generated in `build-mingw/`:

- `libDucoCobotAPI.a`: SDK static library.
- `duco_move.exe`: Left/right arm example.
- `duco_components.exe`: Waist/head example.
- `duco_component_rpc_test.exe`: Loopback test executable.

At runtime, the runtime DLLs from the same MinGW toolchain must be available through `PATH`. The script currently builds with `-O0 -g`.

### Linux

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DDUCO_BUILD_EXAMPLES=ON
cmake --build build --parallel
```

These command forms require CMake 3.13 or later. The project's declared minimum version is 3.5. With older versions, manually create and enter the `build` directory, then run `cmake ..` and `cmake --build .`.

The default library output is `build/libDucoCobotAPI.so`; examples are placed in `build/examples/`. `DUCO_BUILD_EXAMPLES` is disabled by default and can be omitted when building only the SDK. A complete Linux build has not yet been verified in this environment.

## Connecting to a controller

| Component | Client class | Example port |
| --- | --- | --- |
| Left arm | `DucoAPI::DucoCobot` | `7003` |
| Right arm | `DucoAPI::DucoCobot` | `7004` |
| Waist | `DucoAPI::DucoCobotWaist` | `7020` |
| Head | `DucoAPI::DucoCobotHead` | `7021` |

Use the actual controller IP address and ensure that its TCP ports are reachable. Each component uses a separate client, and the client type must match the service. The ports above are example settings; use the controller's actual configuration.

Constructing a client does not establish a connection; call `open()` explicitly. The constructor sets a 3000 ms connection timeout. Ports above `9000` use `TFramedTransport`; other ports use `TBufferedTransport`. Both use `TBinaryProtocol`.

### Minimal example: connect and read joint positions

Save the following code as `main.cpp` and replace the IP address with the actual controller address:

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

Compile with Windows MinGW using the static library built above:

```powershell
g++ -std=c++11 -Isrc main.cpp build-mingw/libDucoCobotAPI.a -pthread -lws2_32 -o robot_app.exe
.\robot_app.exe
```

Compile and run on Linux:

```bash
g++ -std=c++11 -Isrc main.cpp -Lbuild -lDucoCobotAPI -pthread -o robot_app
export LD_LIBRARY_PATH="$PWD/build:${LD_LIBRARY_PATH:-}"
./robot_app
```

The application and SDK must use compatible compiler toolchains, runtime libraries, and target architectures. A MinGW static library cannot be used directly as an MSVC library.

## API conventions

| Interface or parameter | Meaning |
| --- | --- |
| `open()`, `close()` | `0` for success, `-1` for failure; do not make further RPC calls after a connection failure |
| Task call with `block=true` | Returns the task's final state |
| Task call with `block=false` | Returns a task ID; query it with `get_noneblock_taskstate(id)` |
| Reference parameters in query methods | Receive output data, for example `get_actual_joints_position(joints)` |
| `get_version()` | Returns the client SDK version |
| `get_robot_version(version)` | Writes the controller version to the output parameter |

Task states are defined in `DucoAPI::TaskState`. A motion without blending normally returns `ST_Finished` (value `4`) on successful completion. Do not interpret this using the connection methods' `0`-means-success convention. Blended motion may return `ST_Interrupt`; follow the documentation for the specific method. Connection methods return their own error codes; other RPC communication errors generally propagate as exceptions.

### Units and parameters

| Data or interface | Units or meaning |
| --- | --- |
| Joint position, velocity, acceleration | rad, rad/s, rad/s²; vector length must match the actual joint count |
| Cartesian pose | `[x,y,z,rx,ry,rz]`: position in m, orientation in rad |
| `v/a` in `movej`, `movej_pose` | Percentages of the system's configured velocity and acceleration |
| `v/a` in `movej2`, `movej_pose2` | rad/s, rad/s² |
| `v/a` in `movel` | m/s, m/s² |
| Blending parameter `r` | `0` disables blending; units vary by method, as documented in the headers |
| `tool/wobj` in motion methods | Usually coordinate system names; empty strings use the current configuration |
| `tool/wobj` in `cal_fkine/cal_ikine` | Numeric vectors; empty vectors use the current configuration |

The arm enable method is `enable(block)`; the waist and head use `enable(axis_num, block)`. Axis numbering and valid values are defined by the controller. Coordinate system setters affect subsequent motion. `set_tool_data_workcell` and `set_wobj_workcell` also update the project file.

### Connections and heartbeats

The main application flow should own connections and reuse them across operations, closing them together when the application workflow ends. Do not reconnect for every motion or call the same client concurrently from multiple threads.

When a heartbeat is needed, establish a separate connection to the same component and periodically call `rpc_heartbeat(time)` from a dedicated thread. The parameter is in ms and defaults to `1000`. The main thread should check and handle worker errors. Join the heartbeat thread before closing connections during shutdown. See `Move.cpp` for an example.

A connection timeout is not an RPC send/receive timeout. The current high-level wrappers do not expose send/receive timeout settings, so calls or thread cleanup may wait for a response during network failures. `close()` does not itself issue a stop, disable the drives, power off, or reset outputs. Heartbeat disconnection protection is handled by the controller; applications that need to stop motion should explicitly call the appropriate stop method.

## Running the examples

### Left/right arm example

Configure the following constants at the top of [Move.cpp](examples/Move.cpp):

```cpp
const string ROBOT_IP = "192.168.120.225";
const unsigned int LEFT_ARM_PORT = 7003;
const unsigned int RIGHT_ARM_PORT = 7004;
const bool RUN_MOTION = false;
```

Rebuild, then run without port or execution arguments:

```powershell
.\build-mingw\duco_move.exe
```

On Linux, run `./build/examples/duco_move`. The program completes the left arm workflow before starting the right arm workflow. Both connections remain open until the entire workflow ends. By default, it only queries state, computes forward/inverse kinematics, and reads tool I/O.

To enable motion, set `RUN_MOTION` to `true` and rebuild. The program maintains separate heartbeat connections for both arms, then enables and executes a blocking motion on each arm in sequence. Each arm's first joint moves by `+0.02 rad` from its current position, at `0.1 rad/s` with an acceleration of `0.2 rad/s²`. Power on according to the site's operating procedure and verify the target and workspace before running. When editing motion code, check the actual joint count. The motion branch has not yet been run for validation.

### Waist/head example

Set `ROBOT_IP`, `WAIST_PORT`, and `HEAD_PORT` at the top of [Components.cpp](examples/Components.cpp), rebuild, and run:

```powershell
.\build-mingw\duco_components.exe
```

On Linux, run `./build/examples/duco_components`. This example only queries waist and head state; it does not execute motion. Both examples accept a single optional command-line argument to override the IP address.

## Further information

- Method parameters, defaults, and return values: see the Chinese comments in `src/DucoCobot*.h`.
- Local loopback tests: add `-RunTests` to the Windows build; on Linux, build with `-DDUCO_BUILD_TESTS=ON` and run CTest.
- Upstream Thrift documentation and licensing information: [Thrift README](docs/thrift-README.md).
