#!/bin/sh

## Print shortcut manual
shortcut_help() {
    default="${COLCON_PREFIX_PATH:-$HOME/ros2_ws/install}/.."
    dir="${1:-$default}"
    mkdir -p "$dir"
    awk '
        /^## / {
            sub(/^## +/,"",$0);
            print "\n\033[1m\033[34m" $0 "\033[0m";
        }
        /^# / {
            sub(/^# +/,"",$0);
            sub("\[","\033[33m[",$0);
            sub("\]","]\033[0m",$0);
            print $0;
        }
        /^alias/ && !/="shortcut/ {
            sub(/^.*="/,"",$0);
            sub(/"$/,"",$0);
            print "\033[35m" "Command: " "\033[0m" $0 "\033[0m";
        }
        /^ *eval/ {
            sub(/^ *eval "/,"",$0);
            sub(/"$/,"",$0);
            gsub("\${","\033[36m${",$0);
            gsub("\}","}\033[0m",$0);
            print "\033[35m" "Command: " "\033[0m" $0 "\033[0m";
        }
        /^alias / {
            sub(/=.+$/,"",$2);
            print "\033[32m" "Shortcut: " "\033[0m\033[1m" $2 "\033[0m";
        }
    ' "${dir}/src/mep3/docker/config/shortcuts.sh"
}
alias h="shortcut_help"

## Build current directory using colcon
# Arguments:
#   - working directory [optional]
shortcut_colcon_workspace_build() {
    dir="${1:-$HOME/ros2_ws}"
    mkdir -p "$dir"
    eval "cd ${dir} && colcon build --symlink-install"
}
alias cb="shortcut_colcon_workspace_build"

## Remove existing build files
# Arguments:
#   - workspace directory [optional]
shortcut_remove_ros_workspace_build() {
    default="${COLCON_PREFIX_PATH:-$HOME/ros2_ws}/.."
    dir="${1:-$default}"
    mkdir -p "$dir"
    eval "rm -rf ${dir}/build/ ${dir}/install/"
}
alias rr="shortcut_remove_ros_workspace_build"

## Source current ROS2 workspace
# Arguments:
#   - workspace directory [optional]
shortcut_source_ros_workspace() {
    default="${COLCON_PREFIX_PATH:-$HOME/ros2_ws/install}/.."
    dir="${1:-$default}"
    mkdir -p "$dir"
    eval "source ${dir}/install/local_setup.bash"
}
alias s="shortcut_source_ros_workspace"

## Launch mep3_simulation
alias sim="ros2 launch mep3_simulation simulation_launch.py"

## Launch mep3_bringup
alias br="ros2 launch mep3_bringup simulation_launch.py"

## Launch Rviz for mep3_bringup
alias rv="ros2 launch mep3_bringup rviz_launch.py"

## Open Webots world
# Arguments:
#   - workspace directory [optional]
#   - world filename
shortcut_webots_open_world() {
    default="${COLCON_PREFIX_PATH:-$HOME/ros2_ws/install}/.."
    if [ -z "$1" ]; then
        dir="${default}"
        file="eurobot_2022.wbt"
    elif echo "$1" | grep '.wbt$'; then
        dir="${default}"
        file="$1"
    else
        dir="$1"
        file="${2:-eurobot_2022.wbt}"
    fi
    mkdir -p "$dir"
    eval "webots ${dir}/src/mep3/mep3_simulation/webots_data/worlds/${file}"
}
alias we="shortcut_webots_open_world"

## Launch NavigateToPose action
# Arguments:
#   - namespace [optional]
#   - position x [m]
#   - position y [m]
#   - angle theta [deg]
shortcut_action_navigate_to_pose() {
    if echo "$1" | grep -qv '^[0-9\.-]*$'; then
        namespace="${1:-big}"
        shift
    else
        namespace='big'
    fi
    x="${1:-0}"
    y="${2:-0}"
    theta="${3:-0}"
    theta="$(echo "${theta} * 3.141592654 / 180.0" | bc -l)"
    position="$(printf '{x: %.3f, y: %.3f, z: 0}' "${x}" "${y}")"
    orientation="$(printf '{x: 0, y: 0, z: 1, w: %.5f}' "${theta}")"
    message="{pose:{header:{frame_id: 'map'},pose:{position:${position},orientation:${orientation}}}}"
    eval "ros2 action send_goal /${namespace}/navigate_to_pose nav2_msgs/action/NavigateToPose '${message}'"
}
alias np="shortcut_action_navigate_to_pose"

## Launch DynamixelCommand action
# Arguments:
#   - namespace [optional]
#   - motor_name
#   - position [deg]
#   - velocity [deg/s]
#   - tolerance [deg]
#   - timeout [s]
shortcut_action_dynamixel() {
    if echo "$2" | grep -qv '^[0-9\.-]*$'; then
        namespace="${1:-big}"
        shift
    else
        namespace='big'
    fi
    motor_name="${1}"
    position="${2:-0}"
    velocity="${3:-90}"
    tolerance="${4:-2}"
    timeout="${5:-2}"
    message="{position: ${position},velocity: ${velocity},tolerance: ${tolerance},timeout: ${timeout}}"
    eval "ros2 action send_goal /${namespace}/dynamixel_command/${motor_name} mep3_msgs/action/DynamixelCommand '${message}'"
}
alias dy="shortcut_action_dynamixel"
