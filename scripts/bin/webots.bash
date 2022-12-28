#!/usr/bin/env bash
# ====================================================================================================================================
# @file       webots.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Sunday, 6th March 2022 7:30:26 pm
# @modified   Wednesday, 28th December 2022 8:33:42 pm
# @project    ros-common
# @brief      Installation script for Webots simulator
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Guarantee that the `bash-utils` project is sourced in the calling shell
if $([[ -z ${BASH_UTILS_HOME:+x} ]]); then
    echo -e \
            "[ERROR] BASH_UTILS_HOME variable is not defined or does not point to the root directory of bash-utils\n" \
            "       project. Please source source_me.bash file in the root directory of this project to provide\n"    \
            "       shell with required dependencies"
    exit 1
fi

# Source bash-utils library
source $BASH_UTILS_HOME/source_me.bash

# ============================================================== Usage ============================================================= #

# Description of the script
declare cmd_description="Installs Webots simulator via APT package"

# ========================================================== Configruation ========================================================= #

# Logging context of the script
declare LOG_CONTEXT="webots"

# Default apt key URL
declare APT_KEY='https://cyberbotics.com/Cyberbotics.asc'
# Default apt respository URL
declare APT_REPO='https://cyberbotics.com/debian/'

# ============================================================== Main ============================================================== #

function install() {

    # Check if Webots is installed already
    is_pkg_installed webots && return 0

    log_info "Adding Webots repository to apt..."

    # Add apt key
    wget -qO- $APT_KEY | sudo apt-key add -
    # Add apt repository
    sudo apt-add-repository "deb $APT_REPO binary-amd64/"
    
    log_info "Updating apt..."

    # Update apt
    sudo apt-get update
        
    # Install the simulator
    install_pkg --su -y -v webots
    
}

# ============================================================== Main ============================================================== #

function main() {

    # Set help generator's configuration
    ARGUMENTS_DESCRIPTION_LENGTH_MAX=120
    # Parsing options
    declare -a PARSEARGS_OPTS
    PARSEARGS_OPTS+=( --with-help )
    PARSEARGS_OPTS+=( --verbose   )
    
    # Parsed options
    parse_arguments
    # If help requested, return
    if [[ $ret == '5' ]]; then
        return
    elif [[ $ret != '0' ]]; then
        return $ret
    fi

    # Run installation routine
    install

}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
