1 QT SDK 5.0 Installation

#	http://www.wikihow.com/Install-Qt-SDK-on-Ubuntu-Linux

** First Qt program

#	http://www.wikihow.com/Create-Your-First-Qt-Program-on-Ubuntu-Linux

2 Qtcreator installation in terminal

	$ sudo apt-get install ros-indigo-qt-create	

	$ sudo apt-get install ros-indigo-qt-build

	$ sudo apt-get install qtcreator

3 Prepare the ros environment

	$ mkdir -p ~/ros_ws/src
	$ cd ~/ros_ws/src
	$ catkin_init_workspace
	$ cd ~/ros_ws/
	$ catkin_make
	$ echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
t
3 Create qt_ros template

	$ cd ~/ros_ws/src
	$ catkin_create_qt_pkg qdude

4 Start qtcreator in terminal

	$ qtcreator

5 How to open a ros project in qtcreator?

#	http://xiaoyatec.com/2015/10/13/ros%E5%BC%80%E5%8F%91%E7%8E%AF%E5%A2%83%E4%B9%8Bqt-creator%E4%BA%8C/



=============================================================================================

Possible errors:

	1. CMake Error at /opt/ros/indigo/share/catkin/cmake/catkinConfig.cmake:75 (find_package):
	  Could not find a package configuration file provided by "qt_build" with any
	  of the following names:

	    qt_buildConfig.cmake
	    qt_build-config.cmake

	  Add the installation prefix of "qt_build" to CMAKE_PREFIX_PATH or set
	  "qt_build_DIR" to a directory containing one of the above files.  If
	  "qt_build" provides a separate development package or SDK, be sure it has
	  been installed.

#	Solution:

		sudo apt-get install ros-indigo-qt-build

	Referring to:

		http://answers.ros.org/question/172056/failed-to-include-qt-ros-from-qt_build/

	2. catkin_make: command not found

		$ source /opt/ros/indigo/setup.bash

	   to add to your ~/.bashrc:

		echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
		source ~/.bashrc

	3. CMake Error: The source "/home/peng/ros_ws/src/qtros/CMakeLists.txt" does not match the source "/home/peng/ros_ws/src/CMakeLists.txt" used to generate cache.  Re-run cmake with a different source directory.

#	Search

	CMake Error: The source .. does not match the source .. used to generate cache.  Re-run cmake with a different source directory.

#	Solve:
	
		Delete CMakeCache.txt in the folder you executing cmake.

	4. If roscd says similar to roscd: No such package/stack 'beginner_tutorials' , you will need to source the environment setup file like you did at the end of the create_a_workspace tutorial:

		$ source devel/setup.bash

	5. source ... No such dir and file...

#	Solve:

		To set your workspace permanently, open your .bashrc file in a text editor.

		for example -- gedit ~/.bashrc -- and add

		export ROS_PACKAGE_PATH=/your/path/to/workspace:$ROS_PACKAGE_PATH

===============================================================================================

Other links that may useful:

	1. rqt plugin

		http://wiki.ros.org/rqt/Tutorials/Writing%20a%20C%2B%2B%20Plugin

	2. ros ide built by QT Creator & Communication Example

		http://my.phirobot.com/blog/2013-12-ros_ide_qtcreator.htmlc 

	3. project set

		http://blog.csdn.net/zyh821351004/article/details/43672887

