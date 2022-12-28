# ====================================================================================================================================
# @file       foxy.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 4th November 2021 8:55:12 pm
# @modified   Wednesday, 28th December 2022 7:57:13 pm
# @project    ros-common
# @brief      ROS2-Foxy-specific installation routines
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================ Constants =========================================================== #

# URL of the ROS2 Ubuntu repository
declare ROS2_REPO_URL="http://packages.ros.org/ros2/ubuntu"

# ============================================================ Functions =========================================================== #

function get_ros_bin_url() {

    # URL of the ROS2 Foxy binary package (amd64/Ubuntu-Focal, Patch Release 6.1)
    local ROS2_BIN_URL="https://github.com/ros2/ros2/releases/download/release-foxy-20211013/ros2-foxy-20211013-linux-focal-amd64.tar.bz2"

    # Print URL
    echo $ROS2_BIN_URL

}

# ================================================================================================================================== #
