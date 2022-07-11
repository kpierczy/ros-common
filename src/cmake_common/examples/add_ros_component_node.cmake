# ====================================================================================================================================
# @file       add_ros_component_node.cmake
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 12th May 2022 11:27:46 pm
# @modified   Monday, 11th July 2022 4:21:50 pm
# @project    ros-common
# @brief      Usage example of the add_ros_component_node() function
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Linkable dependencies
list(APPEND LIB_DEPENDENCIES
    Boost    
    rclcpp
)

# Add node component (plugin shared library + executable)
add_ros_component_node(

    # Library target
    LIBRARY_NAME ${PROJECT_NAME}_component
    # Plugin target
    PLUGIN_NAME "namespace::NodeClass"
    # Executable target
    EXECUTABLE_NAME ${PROJECT_NAME}_component_node

    # Sources
    SOURCES
        src/node.cpp

    # Ament dependencies
    AMENT_DEPENDENCIES ${LINK_DEPENDENCIES}
    
)
