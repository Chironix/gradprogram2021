# gradprogram2021
<img src='https://global-uploads.webflow.com/6029e95498d11750a14b3e48/602cb317710199017177ecb9_Chironix-logo-r-61%20copy%202-p-500.png' alt="Chironix Logo" />

### ©Chironix, pty ltd.

# finders_keepers Package
**Mohamed Tolba**

## This Repo Contains:
   * The "Finders Keepers" Problem
   * "finders_keepers" Package

## "FINDERS KEEPERS" Problem Statement
  * It is required to create a robotic solution that can autonomously navigate an unknown environment to find as many of the available April tags as possible.

## "finders_keepers" Package Summary
The objective of this package is to autonomously navigate [Jackal](https://www.clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) 
(a CLEARPATH UGV) through an unknown environment while identifying and recording as many of the available April tags as possible.
This package uses SLAM and a custom-made frontier exploration algorithm based on laser data from a 
[Velodyne VLP-16 LIDAR](http://velodynelidar.com/vlp-16.html) for navigation and mapping, and uses [apriltag](https://pypi.org/project/apriltag/) 
package to detect the April Tags.

### Overview
- The [navfn/NavfnROS](http://wiki.ros.org/navfn) planner is used for global path planning.
- The [dwa_local_planner/DWAPlannerROS](http://wiki.ros.org/dwa_local_planner) planner is used for local path planning.
- The [gmapping](http://wiki.ros.org/gmapping) package is used for laser-based SLAM (Simultaneous Localization and Mapping).
- The [apriltag](https://pypi.org/project/apriltag/) package is used for April-Tags detection.
- A custom-made exploration planner algorithm is used to continuously find the robot's next goal position. 
This algorithm is built off the frontier exploration algorithm developed by 
[Brian Yamauchi](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf)
and the algorithm built by [Michael Wiznitzer](https://mechwiz.github.io/Portfolio/jackal.html).

### How it works?
* The _finders_keepers_ package provides _/explore_find_ node which is responsible for finding and filtering frontiers, detecting April Tags, and 
sending movement commands to the [_/move_base_](http://wiki.ros.org/move_base) node.
* When _/explore_find_ node is launched, the robot starts its movement 
by performing a complete rotation around itself while checking for April Tags on the near walls. If a new April Tag is found, 
its id will be identified and stored, and a picture for it will be taken and saved. The number of April Tags detected 
and their ids are published to the custom topic _/april_tags_ _(finders_keepers/tag_msg)_.
* A search for frontiers will then be conducted. It is essentially finding all the boundaries in a map that are between 
  known and unknown areas. The frontier finder developed in this package uses the [global costmap](http://wiki.ros.org/costmap_2d) to find the frontier points, 
  then it publishes them through the custom topic _/frontiers_ _(finders_keepers/frontier_msg)_.
* Then, a multi-layer filtering process for these frontiers is performed to select the closest accessible 
  frontier of size greater than a pre-defined minimum value. After that, the selected frontier is given as the next requried goal
  position to the _/move_base_ node.
* Every time the robot arrives at or close to a given goal position, it stops and performs a complete rotation around itself looking for April Tags. _It must be noted that
the search for new April Tags is a continuous process whenever the robot is in motion_.
* A custom-built algorithm _free_robot()_ is developed to check and free a stuck robot with a given tolerance.
* Once the robot explores all frontiers it gives the user an option whether to get back to its start position or not. After returning to 
the start position, the robot asks the user if he/she wants to restart the exploration process (In case there are other undiscovered frontiers).
* The April Tags identified in the exploration process have their ids recorded in a text file and their pictures taken and saved.

#### A brief summary for the frontiers finder algorithm
* The frontier finder developed in this package uses the [global costmap](http://wiki.ros.org/costmap_2d) to find the frontier points: 
  * The global costmap uses the static layer to make use of the map (published through _/map_ topic) constructed by [/slam_gmapping](http://wiki.ros.org/slam_gmapping) node 
  and uses the inflation layer to inflate the obstacles in the retrieved map.
  * The global costmap is then subscribed to through _/move_base/global_costmap/costmap_ topic. Subsequently, the 
    costmap is converted into a grayscale picture where white color refers to free area, grey color refers to unknown area, and 
    black color refers to an obstacle. Any map cell has a cost above zero is considered an obstacle (black color) while searching for frontiers.
  * After some image processing, frontier points are found to be the edges between the known (white) and unknown (grey) areas.

#### A brief summary for the frontiers filtering algorithm
* A multi-layer filtering process is performed to find the next best frontier candidate, taking in consideration the following aspects:
  * Very small frontiers are rejected.
  * The nearest frontier to the robot is checked by comparing the lengths of the paths from the current robot location to the different frontiers using _/move_base/make_plan_ service.
  * If the nearest frontier is smaller than some specific value, it is tested for accessibility by a custom-built GBFS (Greedy Best First Search) algorithm.
This algorithm checks in a fast way if there is a path through the known free space from the robot location to the location of the candidate frontier:
    * If the frontier is found accessible, it will be sent as the next goal position to the _/move_base_ node.
    * If the frontier is found inaccessible, it will be added to the rejected-frontiers list, and the algorithm will consider another frontier.
    * If the accessibility check couldn't be done, and that happens when the robot is located at a high-cost-value map cell at the moment, the accesiibility of the frontier point is checked
using the last know free robot location (the last robot location in the map where the map-cell cost value is equal to zero).
* After exploring all the frontiers according to the mentioned criteria, the accessibility check is relaxed. 
The frontier points are searched again, but when the GBFS check their accessibility it will consider paths traversing undiscovered areas.

#### A brief summary for the free_robot algorithm
The _free_robot()_ function, when called, is responsible for moving the robot using the laser readings to the nearest location where the nearest obstacle would be no closer
than a given tolerance. It is a multi-layer algorithm as following:
* First, all the lasers readings are checked for the best direction to orient the robot for its next movement step, and the movement direction is determined.
* The best orientation is then given as a required angle to a custom-built function to rotate the robot.
* According to the determined direction, the robot will move forward or backward until it finds itself at a distance equal to or less the required tolerance. At that time,
the laser readings are checked again for a new orientation, and so on until the robot reaches a location where all the laser readings show that the nearest obstacle is 
at least at the given tolerance away.

### Build Instructions
- The repo is to be built on Ubuntu 18, running ROS melodic.
- To install ROS Melodic on your system, please follow the instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu). 
Make sure you install the **ros-melodic-desktop-full** version and don't forget to initialize and update rosdep.
- Now, clone this repository to your desired location and follow the following instructions. For the sake of illustration, the repository is cloned to Desktop.
```bash
cd ~/Desktop
git clone https://github.com/Mohamed-Demerdash/gradprogram2021.git
cd gradprogram2021/
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
Your catkin package for the Chironix Graduate Program should have been built. You must source the package each time before using it in a terminal :
```bash
source ~/Desktop/gradprogram2021/devel/setup.bash
```
Or add the setup file to your bashrc file :
```bash
echo "source ~/Desktop/gradprogram2021/devel/setup.bash" >> ~/.bashrc
```

And then restart terminal.

If you do not have [apriltag](https://pypi.org/project/apriltag/) package installed, you can use the following command to install it :
```bash
pip install apriltag
```
### Usage
Now the simulation is ready to be launched.  There are two versions of the world simulation -- full and lite. 
If your system resources are not rendering satisfactory performance of the full version, please try the lite version. 
In one terminal window use one of the following launch commands: 
```bash
roslaunch jackal_gazebo jackal_full.launch
roslaunch jackal_gazebo jackal_lite.launch
```
Each launch file will launch the gazebo simulation, an instance of rviz and the move base navigation stack as well. 

In a different terminal tab/window, launch the following command line :
```bash
roslaunch finders_keepers user_control.launch
```
This command will launch _/ctrl_jackal_ node that will start an interface for the user with the following possible control commands :
```bash
'explore':   starts /explore_find node
'tags':      shows the ids of the identified april tags
'save_map':  saves the map constructed so far
'stop':      kills the exploration process and stops the robot
'restart':   restarts /move_base and /explore_find nodes
'shutdown':  stops the robot and shuts down
'exit_ctrl': kills the /ctrl_jackal node only
'clear':     clears the terminal
```
Type _'explore'_ to start the exploration process. While running, the exploration process informs the user all the time, through messages,
of its current action. Thus, the user may check any time what the robot is doing at the moment through the newly opened terminal tab
where _/explore_find_ node was launched.

While the exploration is in progress you may check the April Tags detected so far through four different ways:
- Use _'tags'_ command.
- In a different terminal, check the "pics" directory. It is the directory where pictures for the detected tags are saved. 
_Note that each April Tag picture is saved with its name being its identified id_.
```bash
roscd finders_keepers
cd pics
ls
```
- In a different terminal, check the text file named 'tag_ids.txt'. You can find in this file the ids of the found April Tags.
```bash
roscd finders_keepers
cd src
cat tag_ids.txt
```
- In a different terminal, you may listen to the _/april_tags_ topic.
```bash
rostopic echo /april_tags
```
If you want to pause the exploration process you can use 'stop' command. It will kill the _/explore_find_ node and stop the robot.

If you want to shut down the exploration process together with the _/slam_gmapping_ and _/move_base_ nodes, you can use 'shutdown' command. 
While shutting down, you will be given a choice to whether or not to delete the identified tags from both the text file and the pics directory. 
You will also be asked whether to save the constructed map of the environment or not. If yes, the map will be saved in the "map" directory in the package.
```bash
roscd finders_keepers
cd map
ls
```

If at any time you want to save the map created by the robot until that moment, you can simply use _'save_map'_ command.

The _/explore_find_ node can detect when the robot is rotating around itself too much or when the robot is confused. That is done by checking if the robot rotated 3 complete 
rotations or covered more than 10 m distance while it is trying to get to its given goal position. If so, the robot is flagged confused and a recovery process takes place.
However, during the operation if the user felt that the robot is confused and cannot recover from its confusion for long time by itself, he/she may use the _'restart'_ command 
to restart _/move_base_ and _/explore_find_ nodes, and that expectedly solves the problem.

## Future Developments
Future developments of this package may include object recognition algorithms that will give the robot a better perception for its environment leading it to behave in a better fashion around different objects.
Moreover, the robot might be given the ability to recognize humans in the environment, thus adjusting its behavior for a better human-robot interaction. A machine-learning algorithm may be implemented to
help the robot free itself in case it gets stuck in very tight areas.