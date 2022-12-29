#!/usr/bin/env bash
# ====================================================================================================================================
# @file       install.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 4th November 2021 12:41:47 am
# @modified   Thursday, 29th December 2022 1:40:36 am
# @project    bash-utils
# @source     https://docs.ros.org/en/$distro/Installation/Ubuntu-Install-Binary.html
# @source     https://docs.ros.org/en/$distro/Installation/Ubuntu-Install-Debians.html
# @brief      Installation/Uninstallation script for ROS2 
#
# @note This script supports ROS2 Foxy and ROS2 Galactic    
# @note This script assumes that `source_me.bash` script from th project's root directory was previously sourced
#
# @copyright Krzysztof Pierczyk © 2021
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
declare cmd_description="Installs/uninstalls ROS2 foxy edition using apt packages manager"

# Arguments' descriptions
declare -A pargs_description=(
    [action]="action to be performed by the script"
    [src]="installation type: either binary-package based installation or apt-based installation"
)

# Opts' descriptions
declare -A opts_description=(
    [install_path]="installation path of the ROS when 'bin' installation/uninstallation is performed"
    [distro]="distribution to be installed"
    [cleanup]="if set downloaded sources will be removed after installation"
)

# ============================================================ Constants =========================================================== #

# Script's log context
declare LOG_CONTEXT="ros2"
# Default distro of the ROS2
declare ROS2_DEFAULT_DISTRO="galactic"
# Supported distros of the ROS2
declare ROS2_SUPPORTED_DISTRO="foxy | galactic | humble"

# Default destination of the ROS2
declare ROS2_DEFAULT_INSTALLATION_PATH_SCHEME="/opt/ros/\$distro"
# Path to the default configuration of the rosdep
declare ROS2_ROSDEP_DEFAULT_CONFIG_PATH="/etc/ros/rosdep/sources.list.d/20-default.list"

# URL of the ROS2 GPG key
declare ROS2_GPG_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
# Destination of the PGP key
declare ROS2_GPG_PATH="/usr/share/keyrings/ros-archive-keyring.gpg"
# Path to the source file of the ROS2 Ubuntu repository
declare ROS2_APT_SOURCE_PATH="/etc/apt/sources.list.d/ros2.list"

# ============================================================ Functions =========================================================== #

function add_ros_repo() {

    # Authorise ROS2 GPG key
    sudo curl -sSL "$ROS2_GPG_URL" -o "$ROS2_GPG_PATH"
    
    # Create apt source file pointing to the ROS2 Ubuntu repository - prepare source file's content
    local APT_SOURCE_CMD="deb [arch=$(dpkg --print-architecture) signed-by=$ROS2_GPG_PATH] $ROS2_REPO_URL $(source /etc/os-release && echo $UBUNTU_CODENAME) main"
    # Write content to the file
    echo "$APT_SOURCE_CMD" | sudo tee "$ROS2_APT_SOURCE_PATH" > /dev/null

}


function install_ros_pkg() {
    install_pkg -yv --su -U ros-${opts[distro]}-desktop
}


function install_ros_bin() {
    
    # Download folder for binaries
    local download_path="/tmp/ros2_${opts[distro]}.tar.bz2"

    # --------------------------- Dependencies ---------------------------

    # List of dependencies packages
    local dependencies_=(
        libpython3-dev     # Developers tools for python3 
        python3-pip        # PIP packages manager
        python3-catkin-pkg # ROS2 requires v0.4.24 while Ubuntu Foxy provides v0.4.16 by default
    )

    # Install additional dependencies
    install_pkg_list -yv --su -U dependencies_

    # --------------------------- Installation ---------------------------

    # Get directory and basename of the destination folder
    local installation_dir=$(dirname ${opts[install_path]})
    # Prepary output directory
    mkdir -p "$installation_dir"

    # Download and extract ROS2-Foxy binaries
    download_and_extract                \
        --verbose                       \
        --log-target="ROS2 binaries"    \
        --arch-path=$download_path      \
        --extract-dir=$installation_dir \
        $(get_ros_bin_url)

    # Rename destination directory
    mv $installation_dir/ros2-linux ${opts[install_path]}

    # Remove sources
    if is_var_set opts[cleanup]; then
        log_info "Deleting downloaded sources..."
        rm -rf $download_path
        log_info "Sources deleted"
    fi
    
}


function install_ros() {
    
    # If 'pkg' installation, force default installation path
    [[ ${pargs[src]} == "pkg" ]] && 
        opts[install_path]=$(distro=${opts[distro]} eval "echo $ROS2_DEFAULT_INSTALLATION_PATH_SCHEME")

    # --------------------------- Dependencies ---------------------------

    # List of dependencies packages
    local -a dependencies=(
        locales                          # Locales utilities
        software-properties-common       # Software packages-manipulation pack
        curl                             # Utility to download files from online servers
        gnupg2                           # Implementation of the OpenPGP standard (encryption)
        lsb-release                      # Utilities related to distro-specific informations
        python3-rosdep2                  # ROS' dependencies utility
        python3-colcon-common-extensions # `colcon` build system
    )

    # Python dependencies
    local -a ros_python_dependencies=(
        argcomplete # ROS autocompletion tool
        vcstool     # Repository-management
    )

    # ROS dependencies to be skipped
    local -a rosdep_skip_dependencies=(
        cyclonedds 
        fastcdr 
        fastrtps 
        rti-connext-dds-5.3.1 
        urdfdom_headers 
    )
        
    # ---------------------------------- Pre-check ----------------------------------

    # Check if ROS is already installed
    [[ -f ${opts[install_path]}/setup.bash ]] && return
    
    # Check whether current locale supports UTF-8
    locale |
    while read line; do 

        local valid_locale=1

        # Check if a locale configuration supports UTF-8
        if [[ ! "$line" =~ .*UTF-8|.*= ]]; then
            
            # If the dirst uncompatibile setting detected, print error
            if [[ valid_locale == "0" ]]; then
                log_error "Locales does not fully support UTF-8 encodeing:"
                valid_locale=0
            fi
            # Print incompatibile line to the user
            echo $line

            # Print help message
            log_warn "You may try to fix it with following commands:"
            echo "sudo apt update && sudo apt install locales"
            echo "sudo locale-gen en_US en_US.UTF-8"
            echo "sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8"
            echo "export LANG=en_US.UTF-8"

        fi

        # If locale invalid, return
        if [[ valid_locale == "0" ]]; then
            return 1
        fi

    done
    
    # ------------------------------ Preconfigruation -------------------------------

    # Source distros-specific routines
    source $(get_script_dir)/ros/${opts[distro]}.bash

    # Run distro-specific preparation routine
    is_function_defined prepare_ros_installation && 
        prepare_ros_installation
    
    # Add ROS repositories to apt
    add_ros_repo || {
        log_warn "Failed to add ROS2 repository to APT"
        return 1
    }
    
    # Install dependencies
    sudo apt update && install_pkg_list -yv --su dependencies || {
        log_warn "Failed to install ROS2 dependencies"
        return 1
    }

    log_info "Installing ROS2 Python dependencies"
    
    # Install python dependencies
    PIP_FLAGS='-U' pip_install_list ros_python_dependencies || {
        log_warn "Failed to install ROS2 Python dependencies"
        return 1
    }

    log_info "Dependencies installed"
    
    # -------------------------------- Installation ---------------------------------

    log_info "Installing ROS2 to ${opts[install_path]}..."
    
    local ret

    # Install ROS2 package (desktop-version)
    case ${pargs[src]} in
        'pkg' ) install_ros_pkg && ret=$? || ret=$? ;;
        'bin' ) install_ros_bin && ret=$? || ret=$? ;;
    esac
    # Check if installation succeeded
    if [[ $ret != 0 ]]; then
        log_error "Failed to install ROS"
        return 1
    fi
    
    log_info "ROS2 installed"

    # --------------------------- `rosdep` configuration ----------------------------

    # Set variables required by rosdep
    export ROS_PYTHON_VERSION=3
    
    # Initialize rosdep
    log_info "Updating rosdep..."
    [[ -f $ROS2_ROSDEP_DEFAULT_CONFIG_PATH ]] || sudo rosdep init
    rosdep update
    # Install rosdep dependencies
    log_info "Installing ROS2-rosdep dependencies."
    rosdep install                                   \
        --rosdistro=${opts[distro]}                  \
        --from-paths ${opts[install_path]}/share     \
        --ignore-src -y                              \
        --skip-keys="${rosdep_skip_dependencies[0]}"

}


function uninstall_ros() {

    # For binary installation, remove ROS folder
    [[ ${pargs[src]} == "bin" ]] && rm -rf ${opts[install_path]}

    # Uninstall ROS-related apt packages
    sudo apt remove ~nros-${opts[distro]}-* && sudo apt autoremove
}

# ============================================================== Main ============================================================== #

function main() {

    # Arguments
    local -A a_action_parg_def=( [format]="ACTION" [name]="action" [type]="s" [variants]="install | uninstall" )
    local -A    b_src_parg_def=( [format]="SRC"    [name]="src"    [type]="s" [variants]="bin | pkg"           )

    # Opts
    local -A    a_path_opt_def=( [format]="--installation-path" [name]="install_path" [type]="p" [default]=$ROS2_DEFAULT_INSTALLATION_PATH_SCHEME                     )
    local -A  b_distro_opt_def=( [format]="-d|--distro"         [name]="distro"       [type]="s" [default]="$ROS2_DEFAULT_DISTRO" [variants]="$ROS2_SUPPORTED_DISTRO" )
    local -A c_cleanup_opt_def=( [format]="-c|--cleanup"        [name]="cleanup"      [type]="f"                                                                      )

    # Set help generator's configuration
    ARGUMENTS_DESCRIPTION_LENGTH_MAX=120
    # Parsing options
    declare -a PARSEARGS_OPTS
    PARSEARGS_OPTS+=( --with-help )
    PARSEARGS_OPTS+=( --verbose   )

    # Parse arguments
    parse_arguments
    # If help requested, return
    if [[ $ret == $PARSEARGS_HELP_REQUESTED ]]; then
        return
    elif [[ $ret != $PARSEARGS_SUCCESS ]]; then
        return $ret
    fi

    # Evaluate installation path
    opts[install_path]=$(distro=${opts[distro]} eval "echo ${opts[install_path]}")
    opts[install_path]=$(to_abs_path ${opts[install_path]})
    # Change log context
    LOG_CONTEXT="$LOG_CONTEXT-${opts[distro]}"

    # Run command
    case ${pargs[action]} in
        install   ) install_ros   ;;
        uninstall ) uninstall_ros ;;
    esac
    
}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
