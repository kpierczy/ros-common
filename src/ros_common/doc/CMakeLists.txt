# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Monday, 11th July 2022 4:21:50 pm
# @project    ros-common
# @brief      CMake file for `ros_common` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ========================================================== Dependencies ========================================================== #

# Dependencies
find_package(doc_common REQUIRED)

# =============================================================== Docs ============================================================= #

# List source files
file(GLOB_RECURSE RST_SOURCE_FILES LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/*.rst)
file(GLOB_RECURSE PY_SOURCE_FILES LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/src/*.py)
# Compile list of source files
set(DOC_SOURCE_FILES ${RST_SOURCE_FILES} ${PY_SOURCE_FILES})

# Add documentation target (Sphinx)
add_sphinx_doc(sphinx_doc

    # Working directory (Sphinx)
    WORKING_DIR ${PROJECT_SOURCE_DIR}
    # Source directory (Sphinx)
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    # Source files (Sphinx)
    SOURCE_FILES ${DOC_SOURCE_FILES}
    # Output directory (Sphinx)
    OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/doc/sphinx
    # Builders (Sphinx)
    BUILDERS "html"
    # Install dircetory (Sphinx)
    INSTALL_DIR "share/${PROJECT_NAME}/doc/sphinx"
    
)

# ================================================================================================================================== #
