param(
    [string]$Compiler = 'g++.exe',
    [string]$BuildDirectory = '',
    [switch]$RunTests
)

$ErrorActionPreference = 'Stop'
$sdkRoot = Split-Path -Parent $PSScriptRoot
if (-not $BuildDirectory) { $BuildDirectory = Join-Path $sdkRoot 'build-mingw' }
$BuildDirectory = [System.IO.Path]::GetFullPath($BuildDirectory)
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$compilerDirectory = Split-Path -Parent $compilerPath
$archiverPath = Join-Path $compilerDirectory 'ar.exe'
if (-not (Test-Path -LiteralPath $archiverPath)) { throw "Cannot find $archiverPath" }
# GCC invokes compiler helpers and links against its runtime DLLs from this directory.
$originalPath = $env:PATH
$env:PATH = $compilerDirectory + ';' + $env:PATH

function Invoke-BuildTool([string]$Executable, [string[]]$ToolArguments) {
    & $Executable @ToolArguments
    if ($LASTEXITCODE -ne 0) { throw "Build tool failed ($LASTEXITCODE): $Executable" }
}

# Compile the bundled runtime and all three robot clients; server skeleton mains are excluded.
$sources = @(
    'src/thrift/TApplicationException.cpp', 'src/thrift/TOutput.cpp',
    'src/thrift/async/TAsyncChannel.cpp', 'src/thrift/async/TAsyncProtocolProcessor.cpp',
    'src/thrift/async/TConcurrentClientSyncInfo.cpp',
    'src/thrift/concurrency/ThreadManager.cpp', 'src/thrift/concurrency/TimerManager.cpp',
    'src/thrift/concurrency/ThreadFactory.cpp', 'src/thrift/concurrency/Thread.cpp',
    'src/thrift/concurrency/Monitor.cpp', 'src/thrift/concurrency/Mutex.cpp',
    'src/thrift/processor/PeekProcessor.cpp',
    'src/thrift/protocol/TBase64Utils.cpp', 'src/thrift/protocol/TDebugProtocol.cpp',
    'src/thrift/protocol/TProtocol.cpp',
    'src/thrift/transport/TTransportException.cpp', 'src/thrift/transport/TFDTransport.cpp',
    'src/thrift/transport/TSimpleFileTransport.cpp', 'src/thrift/transport/TSocket.cpp',
    'src/thrift/transport/TSocketPool.cpp', 'src/thrift/transport/TServerSocket.cpp',
    'src/thrift/transport/TTransportUtils.cpp', 'src/thrift/transport/TBufferTransports.cpp',
    'src/thrift/transport/TFileTransport.cpp',
    'src/thrift/server/TConnectedClient.cpp', 'src/thrift/server/TServerFramework.cpp',
    'src/thrift/server/TSimpleServer.cpp', 'src/thrift/server/TThreadPoolServer.cpp',
    'src/thrift/server/TThreadedServer.cpp',
    'src/thrift/windows/TWinsockSingleton.cpp', 'src/thrift/windows/SocketPair.cpp',
    'src/thrift/windows/GetTimeOfDay.cpp', 'src/thrift/windows/WinFcntl.cpp',
    'src/thrift/gen-cpp/robot_constants.cpp', 'src/thrift/gen-cpp/robot_types.cpp',
    'src/thrift/gen-cpp/RPCRobot.cpp', 'src/thrift/gen-cpp/RPCHead.cpp',
    'src/thrift/gen-cpp/RPCWaist.cpp',
    'src/DucoCobot.cpp', 'src/DucoCobotHead.cpp', 'src/DucoCobotWaist.cpp'
)
$objectDirectory = Join-Path $BuildDirectory 'obj'
New-Item -ItemType Directory -Path $objectDirectory -Force | Out-Null
$flags = @('-std=c++11', '-O0', '-g', '-pthread', '-DWIN32', '-DTHRIFT_STATIC_DEFINE',
           '-I', (Join-Path $sdkRoot 'src'))
try {
    $objects = @()
    foreach ($source in $sources) {
        $objectPath = Join-Path $objectDirectory (($source -replace '[/.]', '_') + '.o')
        Write-Output "Compile $source"
        Invoke-BuildTool $compilerPath ($flags + @('-c', (Join-Path $sdkRoot $source), '-o', $objectPath))
        $objects += $objectPath
    }
    $libraryPath = Join-Path $BuildDirectory 'libDucoCobotAPI.a'
    Invoke-BuildTool $archiverPath (@('rcs', $libraryPath) + $objects)
    foreach ($example in @(@{ Source = 'Move'; Target = 'move' }, @{ Source = 'Components'; Target = 'components' })) {
        $targetName = 'duco_' + $example.Target + '.exe'
        Write-Output "Link $targetName"
        Invoke-BuildTool $compilerPath ($flags + @((Join-Path $sdkRoot ("examples/" + $example.Source + '.cpp')),
            $libraryPath, '-lws2_32', '-o', (Join-Path $BuildDirectory $targetName)))
    }
    Write-Output 'Link duco_component_rpc_test.exe'
    $testExecutable = Join-Path $BuildDirectory 'duco_component_rpc_test.exe'
    Invoke-BuildTool $compilerPath ($flags + @((Join-Path $sdkRoot 'tests/component_rpc_test.cpp'),
        $libraryPath, '-lws2_32', '-o', $testExecutable))
    if ($RunTests) { Invoke-BuildTool $testExecutable @() }
    Write-Output "Build completed: $BuildDirectory"
    Write-Output "Before running examples, add $compilerDirectory to PATH for the MinGW runtime DLLs."
} finally {
    $env:PATH = $originalPath
}
