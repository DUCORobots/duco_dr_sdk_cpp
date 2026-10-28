cmake_minimum_required(VERSION 3.5)

# 工程名称、语言、版本
project(DucoCobotAPI
  LANGUAGES C CXX
  VERSION 0.1.0
  )

SET(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(BUILD_SHARED_LIBS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
add_definitions(-std=c++11)
include_directories(src)

# Create the thrift C++ library
set( thriftcpp_SOURCES
   src/thrift/TApplicationException.cpp
   src/thrift/TOutput.cpp
   src/thrift/util.h
   src/thrift/async/TAsyncChannel.cpp
   src/thrift/async/TAsyncProtocolProcessor.cpp
   src/thrift/async/TConcurrentClientSyncInfo.h
   src/thrift/async/TConcurrentClientSyncInfo.cpp
   src/thrift/concurrency/ThreadManager.cpp
   src/thrift/concurrency/TimerManager.cpp
   src/thrift/processor/PeekProcessor.cpp
   src/thrift/protocol/TBase64Utils.cpp
   src/thrift/protocol/TDebugProtocol.cpp
   src/thrift/protocol/TProtocol.cpp
   src/thrift/transport/TTransportException.cpp
   src/thrift/transport/TFDTransport.cpp
   src/thrift/transport/TSimpleFileTransport.cpp
   src/thrift/transport/TSocket.cpp
   src/thrift/transport/TSocketPool.cpp
   src/thrift/transport/TServerSocket.cpp
   src/thrift/transport/TTransportUtils.cpp
   src/thrift/transport/TBufferTransports.cpp
   src/thrift/server/TConnectedClient.cpp
   src/thrift/server/TServerFramework.cpp
   src/thrift/server/TSimpleServer.cpp
   src/thrift/server/TThreadPoolServer.cpp
   src/thrift/server/TThreadedServer.cpp
)

# These files don't work on Windows CE as there is no pipe support
# TODO: These files won't work with UNICODE support on windows. If fixed this can be re-added.
if (NOT WINCE)
    list(APPEND thriftcpp_SOURCES
       src/thrift/transport/TFileTransport.cpp
    )
endif()


if (WIN32)
    list(APPEND thriftcpp_SOURCES
        src/thrift/windows/TWinsockSingleton.cpp
        src/thrift/windows/SocketPair.cpp
        src/thrift/windows/GetTimeOfDay.cpp
        src/thrift/windows/WinFcntl.cpp
    )

else()
    # These files evaluate to nothing on Windows, so omit them from the
    # Windows build
    list(APPEND thriftcpp_SOURCES
        src/thrift/VirtualProfiling.cpp
        src/thrift/server/TServer.cpp
    )

    list(APPEND thriftcpp_SOURCES
       src/thrift/gen-cpp/robot_constants.h
       src/thrift/gen-cpp/robot_constants.cpp
       src/thrift/gen-cpp/robot_types.h
       src/thrift/gen-cpp/robot_types.cpp
       src/thrift/gen-cpp/RPCRobot.h
       src/thrift/gen-cpp/RPCRobot.cpp
       src/thrift/gen-cpp/RPCHead.h
       src/thrift/gen-cpp/RPCHead.cpp
       src/thrift/gen-cpp/RPCWaist.h
       src/thrift/gen-cpp/RPCWaist.cpp
         )

     list(APPEND thriftcpp_SOURCES
        src/DucoCobot.h
        src/DucoCobot.cpp
        src/DucoCobotHead.h
        src/DucoCobotHead.cpp
        src/DucoCobotWaist.h
        src/DucoCobotWaist.cpp
          )
endif()

if(UNIX)
    if(ANDROID)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    else()
        list(APPEND SYSLIBS pthread)
    endif()
endif()

set( thriftcpp_threads_SOURCES
    src/thrift/concurrency/ThreadFactory.cpp
    src/thrift/concurrency/Thread.cpp
    src/thrift/concurrency/Monitor.cpp
    src/thrift/concurrency/Mutex.cpp
)



message(${thriftcpp_SOURCES})

add_library(DucoCobotAPI ${thriftcpp_SOURCES} ${thriftcpp_threads_SOURCES})
target_include_directories(DucoCobotAPI PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
if(WIN32)
    target_link_libraries(DucoCobotAPI  ws2_32)
else()
    target_link_libraries(DucoCobotAPI  pthread)
endif()

# Robot examples use the DucoCobot wrapper, currently built by the UNIX branch.
option(DUCO_BUILD_EXAMPLES "Build the robot SDK examples (UNIX)" OFF)
if(DUCO_BUILD_EXAMPLES)
    if(NOT UNIX)
        message(FATAL_ERROR "Robot examples currently require the UNIX SDK build; see README.md")
    endif()
    add_subdirectory(examples)
endif()

# Loopback RPC checks; no robot hardware is required.
option(DUCO_BUILD_TESTS "Build component RPC integration tests (UNIX)" OFF)
if(DUCO_BUILD_TESTS)
    if(NOT UNIX)
        message(FATAL_ERROR "Component RPC tests require the UNIX SDK build")
    endif()
    enable_testing()
    find_package(Threads REQUIRED)
    add_executable(duco_component_rpc_test tests/component_rpc_test.cpp)
    target_link_libraries(duco_component_rpc_test PRIVATE DucoCobotAPI Threads::Threads)
    add_test(NAME duco_component_rpc COMMAND duco_component_rpc_test)
    set_tests_properties(duco_component_rpc PROPERTIES TIMEOUT 30)
endif()


if(WITH_QT5)
    add_subdirectory(src/thrift/qt)
endif()

if(MSVC)
    add_definitions("-DUNICODE -D_UNICODE")
endif()

# Install the headers
install(DIRECTORY "src/thrift" DESTINATION "${INCLUDE_INSTALL_DIR}"
    FILES_MATCHING PATTERN "*.h" PATTERN "*.tcc")
# Copy config.h file
install(DIRECTORY "${CMAKE_BINARY_DIR}/thrift" DESTINATION "${INCLUDE_INSTALL_DIR}"
    FILES_MATCHING PATTERN "*.h")

