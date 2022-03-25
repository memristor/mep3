#!/bin/sh

detect_ros_ws_in_path() {
    pwd_ws="$(
        pwd | awk -F '/' '
            BEGIN {
                ws=1;
            }
            /_ws/ {
                for (i = 1; i <= NF; i++) {
                    if ($i ~ /_ws$/) {
                        ws=i;
                        break;
                    }
                }
            }
            END {
                for (i = 2; i <= ws; i++) {
                    printf "/%s", $i;
                }
            }
        '
    )"
    default_ws="${pwd_ws:-$HOME/ros2_ws}"
}

## Print shortcut manual
shortcut_help() {
    detect_ros_ws_in_path
    default="${COLCON_PREFIX_PATH:-$default_ws/install}/.."
    dir="${1:-$default}"
    mkdir -p "$dir"
    awk '
        /^## / {
            sub(/^## +/,"",$0);
            print "\n\033[1m\033[34m" $0 "\033[0m";
        }
        /^# / {
            sub(/^# +/,"",$0);
            gsub("\\[","\033[33m[",$0);
            gsub("\\]","]\033[0m",$0);
            print $0;
        }
        /^alias/ && !/="shortcut/ {
            cmd=$0
            sub(/^.*="/,"",cmd);
            sub(/"$/,"",cmd);
            print "\033[35m" "Command: " "\033[0m" cmd "\033[0m";
        }
        /^alias / {
            sub(/=.+$/,"",$2);
            print "\033[32m" "Shortcut: " "\033[0m\033[1m" $2 "\033[0m";
        }
        /^ *eval/ {
            sub(/^ *eval "/,"",$0);
            sub(/"$/,"",$0);
            gsub("\\${","\033[36m${",$0);
            gsub("\\}","}\033[0m",$0);
            print "\033[35m" "Command: " "\033[0m" $0 "\033[0m";
        }
    ' "${dir}/src/mep3/docker/config/shortcuts.sh"
}
alias h="shortcut_help"

## Add previous action to scratchpad
shortcut_scratchpad_add_previous() {
    scratchpad="${scratchpad}${last_action}${last_action:+;}"
    last_action=''
}
alias spa="shortcut_scratchpad_add_previous"

## Remove last action from scratchpad
shortcut_scratchpad_remove_last() {
    scratchpad="$(echo ${scratchpad} | sed 's/[^;]*;$//')"
}
alias spr="shortcut_scratchpad_remove_last"

## Clear scratchpad
shortcut_scratchpad_clear() {
    scratchpad=''
}
alias spclr="shortcut_scratchpad_clear"

## Print scratchpad actions as BehaviorTree XML
shortcut_scratchpad_print() {
    echo "${scratchpad}" | tr ';' '\n' | awk '
        $1 == "navigate_to_pose" {
            printf "<Action ID=\"NavigateToAction\" goal=\"%s;%s;%.5f\" />\n", $2, $3, $4;
        }
        $1 == "precise_navigate_to_pose" {
            printf "<Action ID=\"PreciseNavigateToAction\" goal=\"%s;%s;%.5f\" />\n", $2, $3, $4;
        }
        $1 == "dynamixel" {
            printf "<Action ID=\"DynamixelCommandAction\" server_name=\"dynamixel_command/%s\" position=\"%s\" velocity=\"%s\" tolerance=\"%s\" timeout=\"%s\" result=\"0\" />\n", $2, $3, $4, $5, $6;
        }
        $1 == "motion" {
            printf "<Action ID=\"MotionCommandAction\" command=\"%s\" value=\"%.5f\" velocity_linear=\"%s\" acceleration_linear=\"%s\" velocity_angular=\"%s\" acceleration_angular=\"%s\" result=\"success\" />\n", $2, $3, $4, $5, $6, $7;
        }
        $1 == "vacuum_pump" {
            printf "<Action ID=\"VacuumPumpCommandAction\" server_name=\"vacuum_pump_command/%s\" connect=\"%s\" result=\"%s\" />\n", $2, $3, $3;
        }
        $1 == "lift" {
             printf "<Action ID=\"LiftCommandAction\" server_name=\"lift_command/%s\" height=\"%s\" velocity=\"%s\" tolerance=\"%s\" timeout=\"%s\" result=\"0\" />\n", $2, $3, $4, $5, $6;
        }
    '
}
alias spp="shortcut_scratchpad_print"

## Build current directory using colcon
# Arguments:
#   - working directory [optional]
shortcut_colcon_workspace_build() {
    detect_ros_ws_in_path
    dir="${1:-$default_ws}"
    mkdir -p "$dir"
    eval "cd ${dir} && colcon build --symlink-install"
}
alias cb="shortcut_colcon_workspace_build"

## Remove existing build files
# Arguments:
#   - workspace directory [optional]
shortcut_remove_ros_workspace_build() {
    detect_ros_ws_in_path
    default="${COLCON_PREFIX_PATH:-$default_ws/install}/.."
    dir="${1:-$default}"
    mkdir -p "$dir"
    eval "rm -rf ${dir}/build/ ${dir}/install/"
}
alias rr="shortcut_remove_ros_workspace_build"

## Source current ROS2 workspace
# Arguments:
#   - workspace directory [optional]
shortcut_source_ros_workspace() {
    detect_ros_ws_in_path
    default="${COLCON_PREFIX_PATH:-$default_ws/install}/.."
    dir="${1:-$default}"
    mkdir -p "$dir"
    eval "source ${dir}/install/local_setup.bash"
}
alias s="shortcut_source_ros_workspace"

## Launch mep3_simulation
alias sim="ros2 launch mep3_simulation simulation_launch.py"

## Launch mep3_bringup
# Keyword arguments:
#   - bt [bool]
#   - strategy [file name without extension]
alias br="ros2 launch mep3_bringup simulation_launch.py"

## Launch Rviz for mep3_bringup
alias rv="ros2 launch mep3_bringup rviz_launch.py"

## Open Webots world
# Arguments:
#   - workspace directory [optional]
#   - world filename
shortcut_webots_open_world() {
    detect_ros_ws_in_path
    default="${COLCON_PREFIX_PATH:-$default_ws/install}/.."
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

## Launch Teleop Twist Keyboard
# Arguments:
#   - namespace [optional]
shortcut_teleop_twist_keyboard() {
    if echo "$2" | grep -qv '^[0-9\.-]*$'; then
        namespace="${1:-big}"
        shift
    else
        namespace='big'
    fi
    eval "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=${namespace}/cmd_vel"
}
alias te="shortcut_teleop_twist_keyboard"

## Launch NavigateToPose or PreciseNavigateToPose action
# Arguments:
#   - precise [optional]
#   - namespace [optional]
#   - position x [m]
#   - position y [m]
#   - angle theta [deg]
shortcut_action_navigate_to_pose() {
    if [ "$1" = 'precise' ]; then
        prefix="precise_"
        shift
    else
        prefix=""
    fi
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
    orientation="$(
        printf '{x: 0, y: 0, z: %.5f, w: %.5f}' \
        "$(echo "s(${theta} / 2)" | bc -l)" \
        "$(echo "c(${theta} / 2)" | bc -l)"
    )"
    message="{
        pose: {
            header: {frame_id: 'map'},
            pose: {
                position: ${position},
                orientation: ${orientation}
            }
        }
    }"
    last_action="${prefix}navigate_to_pose ${x} ${y} ${theta}"
    eval "ros2 action send_goal /${namespace}/${prefix}navigate_to_pose nav2_msgs/action/NavigateToPose '${message}'"
}
alias np="shortcut_action_navigate_to_pose"
alias pnp="shortcut_action_navigate_to_pose precise"

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
    message="{
        position: ${position},
        velocity: ${velocity},
        tolerance: ${tolerance},
        timeout: ${timeout}
    }"
    last_action="dynamixel ${motor_name} ${position} ${velocity} ${tolerance} ${timeout}"
    eval "ros2 action send_goal /${namespace}/dynamixel_command/${motor_name} mep3_msgs/action/DynamixelCommand '${message}'"
}
alias dy="shortcut_action_dynamixel"

## Launch LynxmotionCommand action
# Arguments:
#   - namespace [optional]
#   - motor_name [optional]
#   - position [deg]
#   - velocity [deg/s]
#   - tolerance [deg]
#   - timeout [s]
shortcut_action_lynxmotion() {
    if echo "$2" | grep -qv '^[0-9\.-]*$'; then
        namespace="${1:-big}"
        shift
    else
        namespace='big'
    fi
    if echo "$1" | grep -qv '^[0-9\.-]*$'; then
        motor_name="${1}"
        shift
    else
        motor_name='lift_motor'
    fi
    position="${1:-0}"
    velocity="${2:-90}"
    tolerance="${3:-2}"
    timeout="${4:-2}"
    message="{
        position: ${position},
        velocity: ${velocity},
        tolerance: ${tolerance},
        timeout: ${timeout}
    }"
    last_action="lynxmotion ${motor_name} ${position} ${velocity} ${tolerance} ${timeout}"
    eval "ros2 action send_goal /${namespace}/lynxmotion_command/${motor_name} mep3_msgs/action/LynxmotionCommand '${message}'"
}
alias ly="shortcut_action_lynxmotion"

## Launch LiftCommand action
# Arguments:
#   - namespace [optional]
#   - height [cm]
#   - velocity [cm/s]
#   - tolerance [cm]
#   - timeout [s]
shortcut_action_lift() {
    if echo "$2" | grep -qv '^[0-9\.-]*$'; then
        namespace="${1:-big}"
        shift
    else
        namespace='big'
    fi
    motor_name='lift_motor'
    height="${1:-0}"
    velocity="${2:-90}"
    tolerance="${3:-2}"
    timeout="${4:-2}"
    position_ly="$(echo "${position} / 15.75" | bc -l)"
    velocity_ly="$(echo "${velocity} / 15.75" | bc -l)"
    tolerance_ly="$(echo "${tolerance} / 15.75" | bc -l)"
    last_action="lift ${height} ${velocity} ${tolerance} ${timeout}"
    eval "shortcut_action_lynxmotion ${namespace} ${motor_name} ${position_ly} ${velocity_ly} ${tolerance_ly} ${timeout}"
}
alias li="shortcut_action_lift"

## Launch MotionCommand action
# Arguments:
#   - namespace [optional]
#   - command [f/r/a]
#   - value [m|deg]
#   - velocity_linear [m/s] [optional]
#   - acceleration_linear [m/s^2] [optional]
#   - velocity_angular [rad/s] [optional]
#   - acceleration_angular [rad/s^2] [optional]
shortcut_action_motion() {
    if echo "$2" | grep -qv '^[0-9\.-]*$'; then
        namespace="${1:-big}"
        shift
    else
        namespace='big'
    fi
    command="${1}"
    value="${2:-0}"
    case "$command" in
        'f'|'forward')
            command='forward';
            velocity_linear="${3:-0}";
            acceleration_linear="${4:-0}";
            velocity_angular=0;
            acceleration_angular=0;;
        'r'|'rotate_relative')
            command='rotate_relative';
            velocity_linear=0;
            acceleration_linear=0;
            velocity_angular="${3:-0}"
            acceleration_angular="${4:-0}"
            value="$(echo "${value} * 3.14159 / 180.0" | bc -l | xargs printf '%.5f')";;
        'a'|'rotate_absolute')
            command='rotate_absolute';
            velocity_linear=0;
            acceleration_linear=0;
            velocity_angular="${3:-0}"
            acceleration_angular="${4:-0}"
            value="$(echo "${value} * 3.14159 / 180.0" | bc -l | xargs printf '%.5f')";;
        *)
            return 1;;
    esac
    message="{
        command: ${command},
        value: ${value},
        velocity_linear: ${velocity_linear},
        acceleration_linear: ${acceleration_linear},
        velocity_angular: ${velocity_angular},
        acceleration_angular: ${acceleration_angular}
    }"
    last_action="motion ${command} ${value} ${velocity_linear} ${acceleration_linear} ${velocity_angular} ${acceleration_angular}"
    eval "ros2 action send_goal /${namespace}/motion_command mep3_msgs/action/MotionCommand '${message}'"
}
alias mo="shortcut_action_motion"

## Launch VacuumPumpCommand action
# Arguments:
#   - namespace [optional]
#   - pump_name
#   - connect [bool]
shortcut_action_vacuum_pump() {
    if echo "$2" | grep -qv '^[0-9\.-]*$'; then
        namespace="${1:-big}"
        shift
    else
        namespace='big'
    fi
    pump_name="${1}"
    connect="${2:-1}"
    message="{
        connect: ${connect}
    }"
    last_action="vacuum_pump ${pump_name} ${connect}"
    eval "ros2 action send_goal /${namespace}/vacuum_pump_command/${pump_name} mep3_msgs/action/VacuumPumpCommand '${message}'"
}
alias vc="shortcut_action_vacuum_pump"
