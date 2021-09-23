# cs393r_starter

## Using this Repo
You will be using a duplicate of this repo for all development in the class. This repo provides starter code for the ROS subscriptions, publishers and control loops you will be using.

### Prerequisites

Please refer to the [UT AUTOmata reference manual]() for instructions on setting up the dependencies:
* [ROS](http://wiki.ros.org/ROS/Installation)
* [amrl_msgs](https://github.com/ut-amrl/amrl_msgs)
* [amrl_maps](https://github.com/ut-amrl/amrl_maps)
* [ut_automata](https://github.com/ut-amrl/ut_automata)

### Duplicate the Repo
1. Make sure you're logged into your GitHub account.
2. Create a new repo with the same name under your GitHub account. Dont initialize that with anything.
3. `git clone <this repository url>` (found in the upper right)
4. `cd <cloned_repo>`
5. `git push --mirror <your new repository url>`

### Clone and Build
1. `git clone <your repository url>` (found in the upper right)
2. After you have cloned the repo, add it to your ROS path by adding the following line to the end of `~/.bashrc`:   
    ```
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:[Local path to your repo]
    ```
    For example:
    ```
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/joydeepb/cs393r_starter
    ```
3. Reload `.bashrc` so that the path is updated:
    ```
    source ~/.bashrc
    ```
4. `cd <cloned_repo>`
5. `make -j`

### Code Overview
There are three main executables: `navigation`, `particle_filter`, and `slam`. Each executable has a corresponding `.h` and `.cc` file that defines the class for the implementation. An associated `*_main.cc` file abstracts away ROS-specific details. For example, the `particle_filter` executable consists of three files:
```
src
└── particle_filter
    ├── particle_filter.cc
    ├── particle_filter.h
    └── particle_filter_main.cc
```
Every header file includes documentation in comments for the variables and subroutines.  
The project compiles with [Eigen](https://eigen.tuxfamily.org/) for linear algebra and coordinate geometry, the [amrl_shared_lib](https://github.com/ut-amrl/amrl_shared_lib) for commonly used robotics subroutines, and a custom simple priority queue implementation. Some useful references on how to use the libraries:
* See the included [`eigen_tutorial.cc`](src/eigen_tutorial.cc) file for example most common Eigen usage, and the [official Eigen tutorials](https://eigen.tuxfamily.org/dox/GettingStarted.html) for more extensive documentation.
* See the included [`simple_queue_test.cc`](src/navigation/simple_queue_test.cc) for usage of the simple priority queue.

### Running the Code
Make sure you recompile your code between changes.
* To run navigation:
    ```
    ./bin/navigation
    ```
* To run the particle filter:
    ```
    ./bin/particle_filter
    ```
* To run SLAM:
    ```
    ./bin/slam
    ```
