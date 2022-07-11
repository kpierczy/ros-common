# Welcome to ros-common library!

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![version](https://img.shields.io/badge/version-1.0.0-blue)

On the contrary to the first edition of [ROS](https://www.ros.org/) (*Robot Operating System*) framework, ROS2 
leverages various characteristics of utilities it depends on to provide cleaner, more modern APIs that makes 
packages both easier to develop and maintain. However, there are still some areas where these interfaces may
seem to be unnecessarily complex, anachronistic or just difficult to remember and use on a regular basis. 
**ros-common** is a set of general use tools that aim to wrap such APIs into more elegant, safe and easier to use
form that makes daily work of roboticist just more pleasant.

The project tries to provide utilities covering all common fields of ROS2 packages development including:

  - writing CMake scripts
  - automatic documentation generation
  - launch system development
  - both C++ and Python-based nodes design
