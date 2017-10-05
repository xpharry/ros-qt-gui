# Conquering Log

## 1. QT SDK 5.0 Installation

http://www.wikihow.com/Install-Qt-SDK-on-Ubuntu-Linux

*First Qt program*

http://www.wikihow.com/Create-Your-First-Qt-Program-on-Ubuntu-Linux

# 2. Qtcreator installation in terminal

```
$ sudo apt-get install ros-indigo-qt-create	
$ sudo apt-get install ros-indigo-qt-build
$ sudo apt-get install qtcreator
```

## 3. Prepare the ros environment

```
$ mkdir -p ~/ros_ws/src
$ cd ~/ros_ws/src
$ catkin_init_workspace
$ cd ~/ros_ws/
$ catkin_make
$ echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
```

## 4. Create qt_ros template

```
$ cd ~/ros_ws/src
$ catkin_create_qt_pkg qdude
```

## 5. Start qtcreator in terminal

```
$ qtcreator
```

## 6. How to open a ros project in qtcreator?

http://xiaoyatec.com/2015/10/13/ros%E5%BC%80%E5%8F%91%E7%8E%AF%E5%A2%83%E4%B9%8Bqt-creator%E4%BA%8C/

http://wiki.ros.org/IDEs#QtCreator

#### NOTE:

1) rosbuild

To open a rosbuild ROS package code as a project, use "Open File or Project" and select the CMakeLists.txt of your ROS package. Take care to select the "[package_name]/build" directory as the build directory, which is the ROS default. On the next screen click 'Run Cmake' and then Finish. This may not show all the folders such as launch and include in the project tree. If you want to choose the files manually, goto File->New File or Project->Import Project->Import Existing Project and selected to choose all files/folders included in the project.

2)  catkin_make

To open a catkin code as a project, use "Open File or Project" and select the top level CMakeLists.txt of the catkin workspace (e.g. "src/CMakeLists.txt"). Select the catkin build folder as the build directory and 'Run CMake' (in order to enable debugging add following line into arguments edit box: -DCMAKE_BUILD_TYPE=Debug).

Recently this has started to fail, because the main CMakeLists is a symlink to a non writtable location. The workaround is to make a copy to toplevel.cmake instead of using a symlink. And if you want the project to be named something else than "Project" then add a line at the top with "project(MyProjectName)"

To be able to modify all the files in the workspace add those lines in "src/CMakeLists.txt" :

**Add all files in subdirectories of the project in a dummy_target so qtcreator have access to all files**

```
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
```

You may specify the correct catkin devel and install spaces at Projects->Build Settings by providing the following CMake arguments: `-DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install`

3) catkin tools

With the new catkin_tools, there is no longer a top level make file for the whole workspace. Instead, open each package as an individual project in QtCreator. The trick is to set the build folder to ws/build/your_package instead of ws/build as before. 

=============================================================================================

**create rqt plugin pkg**

http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin

	catkin_create_pkg rqt_mypkg roscpp rqt_gui rqt_gui_cpp

**Writing a C++ Plugin**

http://wiki.ros.org/rqt/Tutorials/Writing%20a%20C%2B%2B%20Plugin

**Using .ui file in rqt plugin**

http://wiki.ros.org/rqt/Tutorials/Using%20.ui%20file%20in%20rqt%20plugin

**Add a new ROS publisher node to a QTcreator project which already contains a ROS subscriber node**

http://answers.ros.org/question/212459/add-a-new-ros-publisher-node-to-a-qtcreator-project-which-already-contains-a-ros-subscriber-node/


=============================================================================================

#### useful resources in github:

https://github.com/ros-visualization

https://github.com/ros-visualization/rqt_common_plugins.git

https://github.com/ros-visualization/rqt_robot_plugins.git

http://docs.ros.org/electric/api/qt_tutorials/html/index.html

=============================================================================================

#### Possible errors:

1. CMake Error at /opt/ros/indigo/share/catkin/cmake/catkinConfig.cmake:75 (find_package):

```
Could not find a package configuration file provided by "qt_build" with any of the following names:

qt_buildConfig.cmake
qt_build-config.cmake

Add the installation prefix of "qt_build" to CMAKE_PREFIX_PATH or set "qt_build_DIR" to a directory containing one of the above files.  If "qt_build" provides a separate development package or SDK, be sure it has been installed.
```

**Solve:**

```
sudo apt-get install ros-indigo-qt-build
```

Referring to:

http://answers.ros.org/question/172056/failed-to-include-qt-ros-from-qt_build/

2. catkin_make: command not found

```
$ source /opt/ros/indigo/setup.bash
```

to add to your ~/.bashrc:

```
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

3. CMake Error: The source "/home/peng/ros_ws/src/qtros/CMakeLists.txt" does not match the source "/home/peng/ros_ws/src/CMakeLists.txt" used to generate cache.  Re-run cmake with a different source directory.

**Search:**

	CMake Error: The source .. does not match the source .. used to generate cache.  Re-run cmake with a different source directory.

**Solve:**

	Delete CMakeCache.txt in the folder you executing cmake.

4. If roscd says similar to roscd: No such package/stack 'beginner_tutorials' , you will need to source the environment setup file like you did at the end of the create_a_workspace tutorial:

```
$ source devel/setup.bash
```

5. source ... No such dir and file...

**Solve:**

	To set your workspace permanently, open your .bashrc file in a text editor.

	for example -- gedit ~/.bashrc -- and add

	export ROS_PACKAGE_PATH=/your/path/to/workspace:$ROS_PACKAGE_PATH

6. http://stackoverflow.com/questions/2752352/how-to-add-include-path-in-qt-creator	

===============================================================================================

## Other links that may useful:

1. rqt plugin

	http://wiki.ros.org/rqt/Tutorials/Writing%20a%20C%2B%2B%20Plugin

2. ros ide built by QT Creator & Communication Example

	http://my.phirobot.com/blog/2013-12-ros_ide_qtcreator.htmlc 

3. project set

	http://blog.csdn.net/zyh821351004/article/details/43672887


===================

https://aur.archlinux.org/packages/ros-indigo-qt-gui/

