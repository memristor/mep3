#!/usr/bin/env fish

function set_or_keep -a name value;
    if eval "test -z \$$name";
        set -g "$name" "$value"
    end
end

function set_or_fallback -a name value fallback;
    if test -z "$value";
        set -g "$name" "$fallback"
    else
        set -g "$name" "$value"
    end
end

function detect_ros_ws_in_path;
    set -g pwd_ws (
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
    )
    set_or_fallback default_ws "$pwd_ws" "$HOME/ros2_ws"
end

## Print shortcut manual
function shortcut_help;
    detect_ros_ws_in_path
    set_or_fallback dir "$COLCON_PREFIX_PATH" "$default_ws/install"
    set dir "$dir/.."
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
        /^abbr/ && !/="shortcut/ {
            cmd=""
            for (i=4;i<=NF;i++) {cmd=cmd " " $i}
            sub(/^ "/,"",cmd);
            sub(/"$/,"",cmd);
            print "\033[35m" "Command: " "\033[0m" cmd "\033[0m";
        }
        /^abbr / {
            print "\033[32m" "Shortcut: " "\033[0m\033[1m" $3 "\033[0m";
        }
        /^ *eval/ {
            sub(/^ *eval "/,"",$0);
            sub(/"$/,"",$0);
            gsub("\\${","\033[36m${",$0);
            gsub("\\}","}\033[0m",$0);
            print "\033[35m" "Command: " "\033[0m" $0 "\033[0m";
        }
    ' "$dir/src/mep3/docker/config/fish/shortcuts.fish"
end
alias h="shortcut_help"

## Find TODO tasks in source files
# Arguments:
#   - name [optional]
function shortcut_find_todo_tasks -a name;
    detect_ros_ws_in_path
    set_or_fallback dir "$workdir" "$default_ws"
    mkdir -p "$dir"
    if test -z "$name";
        grep -IHnri --exclude=shortcuts.fish "TODO" "$dir/src/mep3/"
    else
        grep -IHnri --exclude=shortcuts.fish "TODO.*(.*$name.*)" "$dir/src/mep3/"
    end
end
alias todo="shortcut_find_todo_tasks"

## Build current directory using colcon
# Arguments:
#   - working directory [optional]
function shortcut_colcon_workspace_build -a workdir;
    detect_ros_ws_in_path
    set_or_fallback dir "$workdir" "$default_ws"
    mkdir -p "$dir"
    eval "cd $dir && colcon build --symlink-install"
end
alias cb="shortcut_colcon_workspace_build"

## Remove existing build files
# Arguments:
#   - workspace directory [optional]
function shortcut_remove_ros_workspace_build -a workspace;
    detect_ros_ws_in_path
    set_or_fallback default "$COLCON_PREFIX_PATH" "$default_ws/install"
    set -g default "$default/.."
    set_or_fallback dir "$workspace" "$default"
    mkdir -p "$dir"
    eval "rm -rf $dir/build/ $dir/install/"
end
alias rr="shortcut_remove_ros_workspace_build"

## Source current ROS2 workspace
# Arguments:
#   - workspace directory [optional]
function shortcut_source_ros_workspace -a workspace;
    detect_ros_ws_in_path
    set_or_fallback default "$COLCON_PREFIX_PATH" "$default_ws/install"
    set -g default "$default/.."
    set_or_fallback dir "$workspace" "$default"
    mkdir -p "$dir"
    if test -f "$dir/install/local_setup.bash";
        exec bash -c "source $dir/install/local_setup.bash; exec bash"
    else
        echo 'Workspace setup file is missing! Build it first.' 1>&2
    end
end
alias s="shortcut_source_ros_workspace"

## Launch mep3_simulation
abbr --add sim "ros2 launch mep3_simulation simulation_launch.py"

## Launch mep3_bringup
# Keyword arguments:
#   - bt [bool]
#   - strategy [file name without extension]
abbr --add br "ros2 launch mep3_bringup simulation_launch.py"

## Launch Rviz for mep3_bringup
abbr --add rv "ros2 launch mep3_bringup rviz_launch.py"

## Open Webots world
# Arguments:
#   - workspace directory [optional]
#   - world filename
function shortcut_webots_open_world -a one two;
    detect_ros_ws_in_path
    set_or_fallback default "$COLCON_PREFIX_PATH" "$default_ws/install"
    set -g default "$default/.."
    if [ -z "$one" ];
        set dir "$default"
        set file "eurobot.wbt"
    else if echo "$one" | grep '.wbt$';
        set dir "$default"
        set file "$one"
    else
        set dir "$two"
        set_or_fallback file "$two" "eurobot.wbt"
    end
    mkdir -p "$dir"
    eval "webots $dir/src/mep3/mep3_simulation/webots_data/worlds/$file"
end
alias we="shortcut_webots_open_world"

## Launch Teleop Twist Keyboard
# Arguments:
#   - namespace [optional]
function shortcut_teleop_twist_keyboard -a namespace;
    if echo "$2" | grep -qv '^[0-9\.-]*$';
        set_or_fallback namespace "$namespace" "$big"
        shift
    else
        set namespace 'big'
    end
    eval "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=$namespace/cmd_vel"
end
alias te="shortcut_teleop_twist_keyboard"


## Launch NavigateToPose action
# Arguments:
#   - namespace
#   - position x [m]
#   - position y [m]
#   - angle theta [deg]
function shortcut_action_navigate_to_pose -a namespace x y theta;
    set theta       (math "$theta * pi / 180.0")
    set z           (math "sin($theta / 2)")
    set w           (math "cos($theta / 2)")
    set position    (printf '{x: %.3f, y: %.3f, z: 0}' $x $y)
    set orientation (printf '{x: 0, y: 0, z: %.5f, w: %.5f}' $z $w)
    set message "   {
        pose: {
            header: {frame_id: 'map'},
            pose: {
                position: $position,
                orientation: $orientation
            }
        }
    }"
    eval "ros2 action send_goal /$namespace/navigate_to_pose nav2_msgs/action/NavigateToPose '$message'"
end
alias np="shortcut_action_navigate_to_pose"

## Launch ScoreboardTaskAction action
# Arguments:
#   - task
#   - points
function shortcut_scoreboard_task -a task points;
    set message "   {
        task: $task,
        points: $points
    }"
    eval "ros2 topic pub --once /scoreboard mep3_msgs/msg/Scoreboard '$message'"
end
alias sc="shortcut_scoreboard_task"

## Launch VacuumPumpCommand action
# Arguments:
#   - namespace
#   - pump_name
#   - connect [bool]
function shortcut_action_vacuum_pump -a namespace pump_name connect;
    set message "   {
        connect: $connect
    }"
    eval "ros2 action send_goal /$namespace/vacuum_pump_command/$pump_name mep3_msgs/action/VacuumPumpCommand '$message'"
end
alias vc="shortcut_action_vacuum_pump"

## Launch MotionCommand action
# Arguments:
#   - namespace
#   - command [f|r|a]
#   - value [m|deg]
#   - velocity [m/s|rad/s] [optional]
#   - acceleration [m/s²|rad/s²] [optional]
function shortcut_action_motion -a namespace cmd val vel accel;
    set_or_fallback value "$val" '0'
    set_or_fallback velocity "$vel" '0'
    set_or_fallback acceleration "$accel" '0'
    switch $cmd
        case 'f'
            set command 'forward'
            set velocity_linear "$velocity"
            set acceleration_linear "$acceleration"
            set velocity_angular 0
            set acceleration_angular 0
        case 'r'
            set command 'rotate_relative'
            set value (math "$value * pi / 180.0")
            set velocity_angular "$velocity"
            set acceleration_angular "$acceleration"
            set velocity_linear 0
            set acceleration_linear 0
        case 'a'
            set command 'rotate_absolute'
            set value (math "$value * pi / 180.0")
            set velocity_angular "$velocity"
            set acceleration_angular "$acceleration"
            set velocity_linear 0
            set acceleration_linear 0
        case '*'
            return 1
    end
    set message "   {
        command: $command,
        value: $value,
        velocity_linear: $velocity_linear,
        acceleration_linear: $acceleration_linear,
        velocity_angular: $velocity_angular,
        acceleration_angular: $acceleration_angular
    }"
    eval "ros2 action send_goal /$namespace/motion_command mep3_msgs/action/MotionCommand '$message'"
end
alias mo="shortcut_action_motion"
