# gradprogram2021
<img src='https://global-uploads.webflow.com/6029e95498d11750a14b3e48/602cb317710199017177ecb9_Chironix-logo-r-61%20copy%202-p-500.png' alt="Chironix Logo" />

### ©Chironix, pty ltd.

This is the master repo for the Chironix 2021 Graduate Program's problem.

# It Contains:
    0. What's this even for?
    1. The "Finders Keepers" problem
    2. The "Creepy Stalker" problem
    3. Some helpful notes, links and hints
    4. Specifics about deployment (i.e your restrictions)


## How did I get here?
    * (www.chironix.com)[Go here.]

## FINDERS KEEPERS
  * Students will create a robotic solution which can autonomously navigate the Chironix lab [An Environment unknown] to find specific objects[Known].
  * Grading will be based on the following criteria:
      [] Reliability of the solution - how often does it crash (program) and how often does it crash (the robot)
      [] Number of objects identified.
      [] Sophistication of object identification solutions (ML/Cameras/features etc)
      [] Number of collisions with obstacles in the environment if any, and, unexplained behaviour vs explained behaviour

## CREEPY STALKER
  * Students will create a tracking solution for one robot to follow another robot - a dynamic task tracking and navigation task in an [Environment unknown] setting.
  * Grading will be based on the following criteria:
      [] Reliability of the solution - how often does it crash (program) and how often does it crash (the robot)
      [] Reliability of the solution to track a given target
      [] Sophistication of object identification solutions (ML/Cameras/features etc)
      [] Number of collisions with obstacles in the environment if any, and, unexplained behaviour vs explained behaviour

## NOTES:
  * Chironix will supply a gazebo environment for the appropriate platform.
  * Chironix will grade entries from DDMMYY to DDMMYY.
  * Announcing finalists between DDMMYY and DDMMYY.
  * Finalists will have a chance to jump on a VC with Chironix Engineers to ensure smooth deployment, this session will be time limited, parties unable to get their entry deployed may be eliminated.
  * Conducting final rounds on our real hardware in our lab from DDDMMYY.
  * Announcing winners DDMMYY.
  * A list of helpful lidar resources is here. [#TODO: Add resources]
  * A list of helpful vision resources. [#TODO: Add resources]
  * URDFs are in the repos.
  * Submit via [PR](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/creating-a-pull-request).
  * Getting started instructions are in the folders enclosed.
  * [ROS Start Guide](http://wiki.ros.org/ROS/StartGuide)
  * [Useful Scout Documentation](https://github.com/agilexrobotics/scout_mini_ros)
  * [Useful Hunter Documentation](https://github.com/agilexrobotics/hunter_ros)
  * [Useful Jackal Documentation](http://wiki.ros.org/Robots/Jackal)
  * [Gazebo Getting started & Installation](http://gazebosim.org/tutorials?tut=quick_start)
  * [Useful Husky Documentation](http://wiki.ros.org/Robots/Husky)
  * The grading rubrick. [#TODO: Create the chart, link it here]

## The potential deployment platforms
    * Clearpath Husky or Jackal, equipped with a realsense camera for odometery, a velodyne 16, [#TODO: Fill in] 
    * AgileX Scout or Hunter, equipped with a realsense camera for odometery, a velodyne 16... [#TODO: Fill in] 

## Build Instructions

The repo is to be built on Ubuntu 18, running ROS melodic.

To install ROS Melodic on your system, please follow the instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu). Make sure you install the ros-melodic-desktop-full version and don't forget to initialize and update rosdep.

Now, clone this repository to your desired location and follow the following instructions. For the sake of illustration, we clone the repository to Desktop.

    cd ~/Desktop
    git clone https://github.com/Chironix/gradprogram2021.git
    cd gradprogram2021/
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
Your catkin package for the Chironix Graduate Program should have been built. You must source the package each time before using it in a terminal : 

    source ~/Desktop/gradprogram2021/devel/setup.bash
Or add the setup file to your bashrc file : 
    
    echo "source ~/Desktop/gradprogram2021/devel/setup.bash" >> ~/.bashrc
And then restart terminal.

Now the simulation is ready to be launched. You are given a choice of 3 robots - Husky, Jackal and Scout mini. There are two versions of the world simulation -- full and lite. If your system resources are not rendering satisfactory performance of the full version, please try the lite version. The launch commands for each robot is as follows : 

    roslaunch husky_gazebo husky_full.launch
    roslaunch husky_gazebo husky_lite.launch
    roslaunch jackal_gazebo jackal_full.launch
    roslaunch jackal_gazebo jackal_lite.launch
    roslaunch scout_gazebo_sim scout_full.launch
    roslaunch scout_gazebo_sim scout_lite.launch
 
 Each launch file will launch the gazebo simulation, an instance of rviz and the move base navigation stack as well. Please note that you may need to customize the move base cost map parameters according to your own need.
