param(
    [ValidateSet('check', 'build', 'test', 'run')][string]$Action = 'check',
    [string]$Port = 'COM3',
    [ValidateSet('velpos', 'odom')][string]$Telemetry = 'velpos',
    [switch]$EnableMotion
)
$ErrorActionPreference = 'Stop'
$bridgeWorkspace = Split-Path $PSScriptRoot -Parent
Push-Location $bridgeWorkspace
try {
    if (-not (Get-Command ros2 -ErrorAction SilentlyContinue)) {
        throw 'Activate the native ROS 2 environment first (Pixi and local_setup.bat). See ros2/README.md.'
    }
    switch ($Action) {
        'check' {
            python -c "import rclpy, serial; from nav_msgs.msg import Odometry; from sensor_msgs.msg import Imu; print('ROS 2 and serial imports OK')"
            if ($LASTEXITCODE -ne 0) { throw 'ROS dependency check failed' }
            python -m serial.tools.list_ports
            if ($LASTEXITCODE -ne 0) { throw 'Port enumeration failed' }
        }
        'build' {
            colcon build --merge-install --base-paths dcaron_bridge --packages-select dcaron_bridge
            if ($LASTEXITCODE -ne 0) { throw 'colcon build failed' }
        }
        'test' {
            python -c "import rclpy, serial, pytest"
            if ($LASTEXITCODE -ne 0) { throw 'Test dependencies missing' }
            colcon test --merge-install --packages-select dcaron_bridge --event-handlers console_direct+
            if ($LASTEXITCODE -ne 0) { throw 'colcon test failed' }
            colcon test-result --verbose
            if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
        }
        'run' {
            if (-not (Test-Path -LiteralPath 'install/local_setup.ps1')) { throw 'Build the package first' }
            . ./install/local_setup.ps1
            $motionValue = if ($EnableMotion) { 'true' } else { 'false' }
            ros2 launch dcaron_bridge bridge.launch.py "port:=$Port" "telemetry:=$Telemetry" "enable_cmd_vel:=$motionValue"
            if ($LASTEXITCODE -ne 0) { throw 'ROS node exited with an error' }
        }
    }
} finally {
    Pop-Location
}
