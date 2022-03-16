#!/bin/sh

# Build current directory using colcon
# Arguments:
#   - working directory [optional]
shortcut_colcon_workspace_build() {
    dir="${1:-$HOME/ros2_ws}"
    mkdir -p "$dir"
    eval "cd ${dir} && colcon build --symlink-install"
}
alias cb="shortcut_colcon_workspace_build"

# Remove existing build files
# Arguments:
#   - workspace directory [optional]
shortcut_remove_ros_workspace_build() {
    default="${COLCON_PREFIX_PATH:-$HOME/ros2_ws}/.."
    dir="${1:-$default}"
    mkdir -p "$dir"
    eval "rm -rf ${dir}/build/ ${dir}/install/"
}
alias rr="shortcut_remove_ros_workspace_build"

# Source current ROS2 workspace
# Arguments:
#   - workspace directory [optional]
shortcut_source_ros_workspace() {
    default="${COLCON_PREFIX_PATH:-$HOME/ros2_ws/install}/.."
    dir="${1:-$default}"
    mkdir -p "$dir"
    eval "source ${dir}/install/local_setup.bash"
}
alias s="shortcut_source_ros_workspace"

# Launch mep3_simulation
alias sim="ros2 launch mep3_simulation simulation_launch.py"

# Launch mep3_bringup
alias br="ros2 launch mep3_bringup simulation_launch.py"

# Launch Rviz for mep3_bringup
alias rv="ros2 launch mep3_bringup rviz_launch.py"

# Open Webots world
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
<<<<<<< Updated upstream
=======

shortcut_action_navigate_to_pose() {
    x="${1:-0}"
    y="${2:-0}"
    theta="${3:-0}"
    theta="$(echo "$theta * 3.141592654 / 180.0" | bc -l)"
    position="$(printf '{x: %.3f, y: %.3f, z: 0}' $x $y)"
    orientation="$(printf '{x: 0, y: 0, z: 1, w: %.5f}' $theta)"
    # echo "position: $position"
    # echo "orientation: $orientation"
    command="ros2 action send_goal /big/navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose:{header:{frame_id: 'map'},pose:{position:$position,orientation:$orientation}}}\""
    echo $command
    eval $command
}
alias np="shortcut_action_navigate_to_pose"
>>>>>>> Stashed changes
