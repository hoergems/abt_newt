Toolkit for approximating and Adapting POMDP solutions In Realtime (TAPIR)
==========================================================================

TAPIR \[1\] is a C++ implementation of the Adaptive Belief Tree (ABT) algorithm \[2\].
ABT is an online POMDP solver capable of adapting to modifications to the POMDP
model without the need to reconstruct the policy from scratch.
For continuous action spaces, TAPIR also includes an implementation of General
Pattern Search in Adaptive Belief Trees (GPS-ABT) \[3\].

\[1\] D. Klimenko and J. Song and. H. Kurniawati.
TAPIR: A Software Toolkit for Approximating and Adapting POMDP Solutions Online.
_Proc. Australasian Conference on Robotics and Automation._
2014.
http://robotics.itee.uq.edu.au/~hannakur/dokuwiki/papers/acra14_tapir.pdf

\[2\] H. Kurniawati and V. Yadav.
An Online POMDP Solver for Uncertainty Planning in Dynamic Environment.
_Proc. Int. Symp. on Robotics Research._
2013.
http://robotics.itee.uq.edu.au/~hannakur/dokuwiki/papers/isrr13_abt.pdf

\[3\] K. Seiler and H. Kurniawati and S.P.N. Singh.
An Online and Approximate Solver for POMDPs with Continuous Action Space.
_Proc. IEEE Int. Conference on Robotics and Automation (ICRA)._
2015.
http://robotics.itee.uq.edu.au/~hannakur/dokuwiki/papers/icra15_ctsActPomdp.pdf

For bug reports and suggestions, please email rdl.algorithm@itee.uq.edu.au

For the latest news, please visit
[the TAPIR website](http://robotics.itee.uq.edu.au/~tapir).


--------------------------------------------------------------------------
TAPIR Development Team
--------------------------------------------------------------------------

- Main developers:
    * Dimitri Klimenko (core toolkit)
    * Konstantin Seiler (continuous action spaces) 
    * Joshua Song (ROS + VREP interface)


--------------------------------------------------------------------------
System Requirements
--------------------------------------------------------------------------

Operating systems: Linux.

Building and running the C++ source code requires:

- [GNU C++ compiler](https://gcc.gnu.org) (>= 4.8) or equivalent
- [libspatialindex](http://libspatialindex.github.io) (>= 1.7.0)
	Debian/Ubuntu package name: "libspatialindex-dev"


**Ubuntu 12.04 NOTE:**

Ubuntu 12.04 by default ships with g++ 4.6. One option is to replace g++ 4.6
with g++ 4.8. Otherwise, to have both g++ 4.8 and g++ 4.6:

    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get install g++-4.8
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 60
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.6 40

Here, 4.8 priority is set to 60, higher than 4.6. To swap to 4.6 use:

    sudo update-alternatives --config g++
    
--------------------------------------------------------------------------
Additional dependencies of the manipulator problems
--------------------------------------------------------------------------
TBA

--------------------------------------------------------------------------
Quick Start (Command-line Interface)
--------------------------------------------------------------------------

TAPIR with command-line interface has been tested with Ubuntu 12.04 -- 14.04.


### Setup and Installation

At the top level directory of TAPIR (where this README file is located), run:

    make all

or, for faster compilation via multi-threading,

    make all -jN

where N is the number of threads you want to use - 8 is a good choice on many
CPUs.

This will create the executables for a command-line interface for each of
the example problems - these are created in `problems/[problem-name]`,
and also in `src/problems/[problem-name]` for convenience while editing the
source files.

### Setup and Installation for the manipulator problems

The two manipulator problems (manipulator_discrete and manipulator_continuous)
require compiling of shared libraries with python bindings. In order to 
compile them, navigate to the /src/problems/shared folder and execute

	make
	
The complied shared libraries are created in /python

### Running

After compiling, you can try the following commands to try using TAPIR on
RockSample, which is a well-known example POMDP:

    cd problems/rocksample
    ./simulate --no-load

This runs the simulator starting with an empty policy, with all of the
calculations being done online during the simulation.
To pre-generate an initial policy and use it as the starting point for
a simulation, you can use the "solve" executable, i.e.

    cd problems/rocksample
    ./solve
    ./simulate

You can also run `RockSample[11,11]`, a version of RockSample with a larger
map and much larger state space, by using the command-line setting
`--cfg default-11-11.cfg`, i.e.

    cd problems/rocksample
    ./simulate --no-load -cfg default-11-11.cfg

The command "solve" uses the parameters set in a problem-specific
configuration file, which for RockSample is, by default,
[here][problems/rocksample/default.cfg].
The purpose of "solve" is to output a policy, which by default will be
written to the file "pol.pol" in the current working directory.

The command "simulate" uses the same default configuration file, but its
function is to run online simulations to evaluate the performance of
the TAPIR solver.
By default it starts by loading an initial policy from the policy file
(default "pol.pol"), but with the argument "--no-load" it begins with an empty
policy instead.
Following this, "simulate" runs a step-by step simulation of the POMDP.
The online solver is run on every step so that the policy can be dynamically
generated on every step.


### Changing Configuration Files

To change the configuration settings, edit the default configuration
files, or copy them and make your own version, e.g.

    cd problems/rocksample
    cp default.cfg my_settings.cfg

Now you can edit my_settings.cfg to change the settings. To use
with a different configuration file, use the command-line argument
--cfg, e.g.

    cd problems/rocksample
    ./solve --cfg my_settings.cfg

Note that "solve" is still searching for configuration settings in
the directory "problems/rocksample" - you can change this by using the argument
`--base-path <path>`.


### Other Options

Some of the core settings can also be set via command-line arguments. Any
command-line options given will override the values specified in the
configuration file. For details, run

    ./solve --help

or

    ./simulate --help

to see the command-line options for either executable.



--------------------------------------------------------------------------
Quick Start (ROS and V-REP Interface)
--------------------------------------------------------------------------


TAPIR provides an interface with ROS and V-REP, tested on:

- Ubuntu 12.04 with ROS Hydro + V-REP PRO EDU V3.1.2.
- Ubuntu 14.04 with ROS Indigo + V-REP PRO EDU V3.1.2.


### Additional system requirements

#### ROS
Go [here](http://wiki.ros.org/hydro/Installation/Ubuntu) for instructions on
how to install ROS. We have tested with ROS Hydro on Ubuntu 12.04 and with ROS
Indigo on Ubuntu 14.04. Some possible issues and ways around them are:

- **Ubuntu 12.04**: ROS Hydro defaults to using Boost 1.46 instead of Boost 1.48, and the Boost
    1.48 headers are incompatible with C++11 (as used by TAPIR).
    To resolve this issue, you will need to install Boost 1.48 from source.
    The easiest way to set up Boost 1.48 for use with ROS and TAPIR is,
    at the top level directory of TAPIR, to do the following:

    1. Set `CUSTOM_BOOST_148_DIR` in the root Makefile to the location
        of your Boost 1.48 installation, or, if you haven't installed it yet,
        the desired directory for it (e.g. `~/vrep`)
    2. Run the command `make boost`

    The command `make boost` will find if a Boost installation is in
    `CUSTOM_BOOST_148_DIR`. If it doesn't find a Boost installation,
    it will automatically download Boost and install it to the directory you've
    specified at `CUSTOM_BOOST_148_DIR`.
    It will also patch a bug in the Boost headers that causes problems
    with GCC >= 4.7.0. These functions are performed by several scripts in the
    `.ros-scripts` directory.

    If you wish to compile Boost 1.48 manually with GCC >= 4.7.0, you should use
    the patch provided [here](https://svn.boost.org/trac/boost/ticket/6165).

- **Ubuntu 14.04**:
    On Ubuntu 14.04 you must instead use ROS Indigo,
    which is available via the package "ros-indigo-desktop-full"


#### V-REP

Go [here](http://www.coppeliarobotics.com/downloads.html), download V-REP, and
extract it to the directory of your choice. TAPIR has been tested with
V-REP PRO EDU V3.1.2.

- **Ubuntu 14.04**:
    On Ubuntu 14.04 the version of the ROS plugin that ships with V-REP will not
    work out of the box - it causes a segmentation fault in V-REP! You will
    need to recompile the plugin.


#### V-REP ROS plugin

V-REP comes with a plugin for ROS, but the default version of this plugin may
not work properly for you - in particular, the current distribution of V-REP
is not designed for ROS Indigo, and so if you're on Ubuntu 14.04 you will need
to rebuild it.

TAPIR comes with a script to automatically recompile the V-REP plugin for you,
which can be run using the command 

    make vrep_plugin
    
Note that this requires you to set the `VREP_DIR` and `ROS_SCRIPT` variables
in the root Makefile before it will work properly.
The script should automatically rebuild the plugin and copy it to `VREP_DIR`.

If you instead wish to manually rebuild the plugin, follow the tutorial
[here](http://www.coppeliarobotics.com/helpFiles/en/rosTutorialHydro.htm).
To do this on Ubuntu 14.04, you will also need to change line 14 of  
`VREP_DIR/programming/ros_packages/vrep_plugin/CMakeLists.txt`
to  
`link_directories("/opt/ros/indigo/lib")`  
instead of  
`link_directories("/opt/ros/hydro/lib")`  
Please note that the V-REP plugin also requires the ROS joystick packages
in order to compile - for ROS Indigo those packages should be available
via the Ubuntu/Debian package `ros-indigo-joy`.


### Setup and installation (for use with ROS and V-REP)

This package is designed to be used with the ROS Catkin build system, and as
such must be compiled within a Catkin workspace. To do this, start at the top
level directory of TAPIR (where this README file is located) and do the following:

1. Set the following variables in the root Makefile :
    - `CUSTOM_BOOST_148_DIR` - A custom path to Boost 1.48; leave this empty
        if you just want to use your system default version of Boost.
    - `ROS_SCRIPT` - path to the main ROS setup script; the default is
        `/opt/ros/hydro/setup.sh`
    - `CATKIN_WS_DIR` - path to the desired Catkin workspace directory;
        the default is `../catkin_ws` (relative to the root tapir directory,
        i.e. the location of this README)
    - `VREP_DIR` - path to where you have extracted V-REP;
        the default is `../vrep`
2. Run the command `make ros`

The command `make ros` can automatically set up a workspace for you and
will add a symbolic link to the source directory into the workspace.


### Running (with ROS and V-REP)

If you compiled with `make ros`, TAPIR will automatically create a script
to run the Tag example problem together with the ROS and V-REP interface. To run
Tag with ROS+VREP, go to the top level directory of TAPIR
(where this README file is located), and run the following commands:

    cd problems/tag
    ./simulate-ros

The script `simulate-ros` will automatically run a *roscore*, and then launch
V-REP (if it is not already running).
Note that the roscore must already be running when you start V-REP,
or the ROS plugin will fail to load; it is also important to read the console
messages when V-REP is staring up in order to make sure that the ROS plugin
loads correctly.
If there is an issue with loading the ROS plugin, you may need to recompile
it - follow the instructions in the
[previous section on the V-REP plugin](#v-rep-ros-plugin).

If you run a roscore and launch V-REP manually, and have sourced the setup
script for your Catkin workspace, i.e.

    source [path/to/catkin/workspace]/devel/setup.bash

you can simply run

    roslaunch tapir tag.launch

in order to start the Tag problem.

After a couple of seconds, the simulation should start automatically; it can
also be paused at any time via the V-REP interface. During the simulation you
can also actively manipulate the environment using the mouse to left-click on
squares in the Tag grid:

- To remove an obstacle, left-click on a cell and wait momentarily.
- To add an obstacle, left-click on a cell and wait momentarily.
- You can also add or remove multiple cells at the same time by holding down
the CTRL key while clicking on those cells.


### Interface details

Additional detail on how to interface with ROS and V-REP when implementing
your own POMDP problems is provided in in the
[ROS/V-REP Guide][docs/ROS_VREP_Guide.md].

--------------------------------------------------------------------------
Implementing a new POMDP model
--------------------------------------------------------------------------

To create a new POMDP model and use it with the the command-line interface,

1. Read the [README on implementing a new model][docs/Making_A_New_Model.md]
2. Create a new subdirectory for your POMDP in
   [the problems directory][src/problems]
3. Implement all the required classes, as per
   [the new model README][docs/Making_A_New_Model.md]
4. Add a new Makefile to your problem directory - just copy
   [the Tag Makefile][src/problems/tag/Makefile] and change MODULE_NAME to match
  your new problem.
5. Add your problem to the CHILD_DIRS variable in the
   [problems Makefile][src/problems/Makefile]
6. Create a new subdirectory in [the config directory][problems] for your
   problem, and add any required configuration files there.
7. Compile your code via `make [problem-name]`, or simply by running
   "make" from your new subdirectory of src/problems, e.g.

        cd src/problems/tag
        make

--------------------------------------------------------------------------
Acknowledgements
--------------------------------------------------------------------------

The development of this package is partially funded by the
University of Queensland BEE Division Grant 2013 and the
BME Early Career Researcher 2014 to Hanna Kurniawati.


--------------------------------------------------------------------------
Release Notes
--------------------------------------------------------------------------




[docs/ROS_VREP_Guide.md]: docs/ROS_VREP_Guide.md
[docs/Making_A_New_Model.md]: docs/Making_A_New_Model.md
[problems]: problems
[problems/rocksample/default.cfg]: problems/rocksample/default.cfg
[src/problems]: src/problems
[src/problems/Makefile]: src/problems/Makefile
[src/problems/tag/Makefile]: src/problems/tag/Makefile
[src/problems/tag/TagModel.hpp]: src/problems/tag/TagModel.hpp
[src/problems/tag/ros/TagVrep.cpp]: src/problems/tag/ros/TagVrep.cpp
[src/solver/Simulator.hpp]: src/solver/Simulator.hpp
