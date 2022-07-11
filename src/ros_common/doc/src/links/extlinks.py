# ====================================================================================================================================
# @file       extlinks.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 25th May 2022 11:48:20 am
# @modified   Monday, 11th July 2022 4:21:50 pm
# @project    ros-common
# @brief      Definitions of extlinks links refering documentations of component packages of the `ros-common` meta-package
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================== Doc =============================================================== #

""" 

.. module:: links.external
   :platform: Unix
   :synopsis: Definitions of extlinks links refering documentations of component packages of the `ros-common` meta-package

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""

# ============================================================= Imports ============================================================ #

from doc_common.sphinx import make_package_extlinks_doc_link

# =========================================================== Definitions ========================================================== #

# List of links to documentations of subpackages
extlinks = {
    **make_package_extlinks_doc_link( 'cmake_common'       ),
    **make_package_extlinks_doc_link( 'doc_common'         ),
    **make_package_extlinks_doc_link( 'launch_common'      ),
    **make_package_extlinks_doc_link( 'node_common_cpp'    ),
    **make_package_extlinks_doc_link( 'package_common_cpp' ),
    **make_package_extlinks_doc_link( 'package_common_py'  ),
}

# ================================================================================================================================== #
