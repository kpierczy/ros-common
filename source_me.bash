#!/usr/bin/env bash
# ====================================================================================================================================
# @file       source_me.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 2nd November 2021 10:18:45 pm
# @modified   Wednesday, 28th December 2022 8:31:35 pm
# @project    bash-utils
# @brief      Script that should be source before starting working with the directory
# @details
#   
#    Script provides user's terminal with definitions required for project's handling. Morover, it checks for project's dependancies
#    and installs them, if needed.
#
#    If the 'update' argument is passed to the script, the `git submodule update --init --recursive` will be run and all system
#    dependencies will be verified and installed as needed. This is optional action as veifying all dependencies may take  a longer 
#    while and it makes no sense to repeat this process at each source action. It is required to run update command at least once 
#    after cloning the repository.
#
#    Optionally, one may source this script with the 'setup' keyword which will make the script to verify dependencies without
#    updating git submodules. This may be handy if some system dependencies are added to the scripts/isntall/*.bash scripts
#    and need to be installed.
#
# @copyright Krzysztof Pierczyk © 2021
# ====================================================================================================================================

# ============================================================== Setup ============================================================= #

# Set project home path
export ROS_COMMON_HOME="$(dirname "$(readlink -f "$BASH_SOURCE")")"

# ============================================================= Helpers ============================================================ #

# Auxiliary log_info used before sourceing `bash-utils`
function log_info_aux()  { echo -e "[\033[32mINFO\033[0m]"  "$@"; }
function log_error_aux() { echo -e "[\033[31mERROR\033[0m]" "$@"; }

# ========================================================== Dependencies ========================================================== #

# Helper function setuping project's system
function setup_project() {

    log_info_aux "Resolving 'bash-utils' dependencies..."

    # Source bash-utils library
    source $ROS_COMMON_HOME/extern/bash-utils/source_me.bash setup || {
        log_error_aux "Failed to resolve 'bash-utils' dependencies"
        return 1
    }

}

# Update project
if [[ "$1" == 'update' ]]; then

    log_info_aux "Clonning git submodules..."

    # Clone submodules
    git submodule update --init --recursive || {
        log_error_aux "Failed to clone git submodules"
        return 1
    }
    
    # Setup project 
    setup_project || return 1
    
# Setup project
elif [[ "$1" == 'setup' ]]; then

    # Setup project 
    setup_project || return 1

fi

# ======================================================== Shell extensions ======================================================== #

# Source bash-utils library
source $ROS_COMMON_HOME/extern/bash-utils/source_me.bash || return 1
# Source bash extensions
source $ROS_COMMON_HOME/scripts/shell/ros.bash || return 1

# ================================================================================================================================== #

# Unset temporary functions
unset -f log_info_aux
unset -f log_error_aux
unset -f setup_project

# ================================================================================================================================== #
