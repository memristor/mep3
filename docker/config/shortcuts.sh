#!/bin/sh

# Build current directory using colcon
# Arguments:
#   - working directory [optional]
shortcut_colcon_workspace_build() {
    dir="${1:-$HOME/ros2_ws}"
    echo "cd ${dir} && colcon build --symlink-install"
}
alias cb="shortcut_colcon_workspace_build"

# Remove existing build files
# Arguments:
#   - workspace directory [optional]
shortcut_remove_ros_workspace_build() {
    default="${COLCON_PREFIX_PATH:-$HOME/ros2_ws}/.."
    dir="${1:-$default}"
    eval "rm -rf ${dir}/install/ ${dir}/build/"
}
alias rr="shortcut_remove_ros_workspace_build"

# Source current ROS2 workspace
# Arguments:
#   - workspace directory [optional]
shortcut_source_ros_workspace() {
    default="${COLCON_PREFIX_PATH:-$HOME/ros2_ws/install}/.."
    dir="${1:-$default}"
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
    echo "webots ${dir}/src/mep3/mep3_simulation/webots_data/worlds/${file}"
}
alias we="shortcut_webots_open_world"
