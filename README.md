[![Build Status](https://travis-ci.org/Sushant-Chavan/coordination_oru.svg?branch=master)](https://travis-ci.org/Sushant-Chavan/coordination_oru)

# A Framework for Multi-Robot Motion Planning, Coordination and Control 

This software implements an _online coordination method for multiple robots_. Its main features are:

* Goals can be posted and paths computed online
* Precedences are inferred online, accounting for robot dynamics via provided dynamic models
* Very few assumptions are made on robot controllers
* The coordination method is not specific to a particular motion planning technique

The software includes a basic 2D robot simulation and a simple built-in motion planner (which depends on the <a href="http://ompl.kavrakilab.org/">OMPL</a> and <a href="http://www.mrpt.org/">MRPT</a> libraries). A <a href="https://github.com/FedericoPecora/coordination_oru_ros">separate interface package</a> is provided to enable the use of this software in conjunction with <a href="http://www.ros.org/">ROS</a> and the <a href="https://github.com/OrebroUniversity/navigation_oru-release">navigation_oru</a> stack to obtain a fully implemented stack for multi-robot coordination and motion planning.

## Overview
The algorithm provided by this implementation is detailed in

* Federico Pecora, Henrik Andreasson, Masoumeh Mansouri, and Vilian Petkov, <a href="https://www.aaai.org/ocs/index.php/ICAPS/ICAPS18/paper/view/17746/16941">A loosely-coupled approach for multi-robot coordination, motion planning and control</a>. In Proc. of the International Conference on Automated Planning and Scheduling (ICAPS), 2018.

[![Examples usages of the coordination_oru library](http://img.youtube.com/vi/jCgrCVWf8sE/0.jpg)](http://www.youtube.com/watch?v=jCgrCVWf8sE "Examples usages of the coordination_oru library")

The approach makes very few assumptions on robot controllers, and can be used with any motion planning method for computing kinematically-feasible paths. Coordination is seen as a high-level control scheme for the entire fleet. Heuristics are used to update precedences of robots through critical sections while the fleet is in motion, and the dynamic feasibility of precedences is guaranteed via the inclusion of user-definable models of robot dynamics. 

The coordination method is based on the _trajectory envelope_ representation provided by the <a href="http://metacsp.org">Meta-CSP framework</a>. This representation is detailed in

* Federico Pecora, Marcello Cirillo, Dimitar Dimitrov, <a href="http://ieeexplore.ieee.org/abstract/document/6385862/">On Mission-Dependent Coordination of Multiple Vehicles under Spatial and Temporal Constraints</a>, IEEE/RSJ International Conference on Intelligent Robots and Systems (2012), pp. 5262-5269.

In short, a trajectory envelope is a set of spatio-temporal constraints on a robot's trajectory. A trajectory envelope spans over a _path_, which is a sequence of _poses_ ```<p1, ... pn>```. In the current implementation, the spatial constraints defining a trajectory envelope are computed as the sweep of the robot's footprint over the path.

## Tutorial
The approach is discussed in detail in the tutorial on _Integrated Motion Planning, Coordination and Control for Fleets of Mobile Robots_, given at the <a href="http://icaps18.icaps-conference.org/tutorials/">2018 International Conference on Automated Planning and Scheduling (ICAPS)</a> by F. Pecora and M. Mansouri. Slides and source code of the tutorial are available <a href="https://gitsvn-nt.oru.se/fopa/coordination-tutorial-src-ICAPS-2018">here</a>.

## Installation
To install, clone this repository:

```
$ git clone https://github.com/FedericoPecora/coordination_oru.git
$ cd coordination_oru
```

Install ROS if not already installed:
```
./install_ros.sh
```

Install dependencies and the coordination_framework:
```
./install.sh
```

## Visualizations
The API provides three visualization methods:

* ```BrowserVisualization```: a browser-based visualization.
* ```JTSDrawingPanelVisualization```: a Swing-based visualization.
* ```RVizVisualization```: a visualization based on the ROS visualization tool <a href="http://wiki.ros.org/rviz">RViz</a>.

All three visualizations implement the abstract ```FleetVisualization``` class, which can be used as a basis to create your own visualization.

Most examples use the ```BrowserVisualization```. The state of the fleet can be viewed from a browser at <a href="http://localhost:8080">http://localhost:8080</a>. The image below shows this visualization for the ```TestTrajectoryEnvelopeCoordinatorThreeRobots``` example:

![BrowserVisualization GUI](images/browser-gui.png "Browser-based visualization")

An arrow between two robots indicates that the source robot will yield to the target robot. Priorities are computed based on a heuristic (which can be provided by the user) and a forward model of robot dynamics (which can also be provided, and is assumed to be conservative - see the <a href="http://iliad-project.eu/wp-content/uploads/papers/PecoraEtAlICAPS2018.pdf">ICAPS 2018 paper</a> mentioned above). The specific poses at which robots yield are also updated online, based on the current positions of robots and the intersecting areas of their trajectory envelopes (critical sections). This makes it possible to achieve "following" behavior, that is, the yielding pose of a robot is updated online while the "leading" robot drives.

The a Swing-based GUI provided by class ```JTSDrawingPanelVisualization``` looks like this:

![Swing-based GUI](images/coord.png "Swing-based visualization")

This GUI allows to take screenshots in SVG, EPS and PDF formats by pressing the ```s```, ```e``` and ```p``` keys, respectively (while focus is on the GUI window). Screenshots are saved in files named with a timestamp, e.g., ```2017-08-13-11:13:17:528.svg```. Note that saving PDF and EPS files is computationally demanding and will temporarily interrupt the rendering of robot movements; SVG screenshots are saved much quicker.

The ```RVizVisualization``` visualization publishes <a href="http://wiki.ros.org/rviz/DisplayTypes/Marker">visualization markers</a> that can be visualized in <a href="http://wiki.ros.org/rviz">RViz</a>. The class also provides the static method ```writeRVizConfigFile(int ... robotIDs)``` for writing an appropriate RViz confiuration file for a given set of robots. An example of the visualization is shown below.

![RVizVisualization GUI](images/rviz-gui.png "RViz-based visualization")

The visualization with least computational overhead is the ```RVizVisualization```, and is recommended for fleets of many robots. The ```BrowserVisualization``` class serves an HTML page with a Javascript which communicates with the coordinator via websockets. Although rendering in this solution is less efficient than in RViz, the rendering occurs on the client platform (where the browser is running), so its computational overhead does not necessarily affect the coordination algorithm. The ```JTSDrawingPanelVisualization``` is rather slow and not recommended for fleets of more than a handful of robots, however it is practical (not requiring to start another process/program for visualization) and relatively well-tested.

## The ```SimpleReedsSheppCarPlanner``` motion planner

A simple motion planner is provided for testing the coordination framework without the need for pre-computed path files. The planner can be used to obtain paths for robots with Reeds-Shepp kinematics (Dubin's car-like robots that can move both forwards and backwards), and is used in several of the included demos.

The provided motion planner depends on the <a href="http://ompl.kavrakilab.org/">Open Motion Planning Library (OMPL)</a>, and the <a href="http://www.mrpt.org/">Mobile Robot Programming Toolkit (MRPT)</a>. The motion planner and its Java interface are purposefully kept very simple. It performs rather poorly in terms of the quality of paths it returns, and is _not_ suited for anything beyond simple examples. Please consider developing a more performing and principled integration with your motion planning software of choice, as done in the <a href="https://github.com/FedericoPecora/coordination_oru_ros">coordination_oru_ros</a> package.

## Installing the ```SimpleReedsSheppCarPlanner``` motion planner

Please install the OMPL and MRPT libraries. Both are present in the official Ubuntu repositories (tested on Ubuntu 16.04):

```
$ sudo apt-get install libompl-dev
$ sudo apt-get install mrpt-apps libmrpt-dev
```

Then, compile and install the ```simplereedssheppcarplanner``` shared library as follows:

```
$ cd coordination_oru/SimpleReedsSheppCarPlanner
$ cmake .
$ make
$ sudo make install
$ sudo ldconfig
```

This will install ```libsimplereedssheppcarplanner.so``` in your ```/usr/local/lib``` directory. A simple JNA-based Java interface to the library is provided in package ```se.oru.coordination.coordination_oru.motionplanning```. The Java class  ```ReedsSheppCarPlanner``` in the same package can be instantiated and used to obtain motions for robots with Reeds-Shepp kinematics.


## Experience Based Planning
The Experience based planning update to the coordination framework allows for storing the previous planning experiences into a database and reuse them during future planning.

### Updates:
* Currently the Lightning and Thunder frameworks provided by OMPL have been implemented in the ```OmplPlanner``` package.
* Many new test cases have been added to test the frameworks in the BRSU and Agaplesion hospital scenarios
* The RViz visualization has been fixed to show the map published on the /map topic
* A graph visualization tool has been developed to display the experiences stored in the databases
* A script has been added to automatically generate training dataset to bootstrap the planning frameworks with prior experiences
* A script has been added to train the frameworks using the training datasets

### Dependencies:
* <a href="https://github.com/Sushant-Chavan/ompl">OMPL 1.4.2</a>
* <a href="https://github.com/Sushant-Chavan/smpl">SMPL</a>

### Usage:
#### Generation of training datasets
Use the ```GenerateTrainingDataset.py``` script to generate training dataset. Example:
```
python3 generators/dataset/GenerateTrainingDataset.py BRSU_Floor0.png --nProblems 25 --dbg_image=True --robot_radius=25 --use_hotspots=True
```

#### Generation of testing datasets
Use the ```GenerateTestingDataset.py``` script to generate testing dataset. Example:
```
python3 generators/dataset/GenerateTestingDataset.py BRSU_Floor0.png --nRobots 5  --dbg_image=True --robot_radius=25
```

#### Generation of Optimality data
Use the ```GenerateOptimalityData.py``` script to generate optimality path length data for the testing problems. Example:
```
python3 generators/dataset/GenerateOptimalityData.py BRSU_Floor0.png --count=5
```

#### Generation of Experiences
Use the ```GenerateExperiences.py``` script to generate experience database using the training problems. For example, to generate the experiences for Lightning framework use:
```
python3 generators/dataset/GenerateExperiences.py BRSU_Floor0.png --count=25 --planner_type=1
```

#### Visualization of the generated experience databases
Use the ```GenerateExperiences.py``` script to visualize the generated experience database
```
python3 graphml_generator/PlotDatabase.py --map_image_filename=BRSU_Floor0.png --count=25
```

#### Launching test executions
Use the ```launchTests.py``` script to iteratively execute the testing use-case. Example:
```
python3 launchTests.py --map=BRSU_Floor0 --nRobots=5 --planner=1 --nExperiences=25 --nIterations=50 --timeout=120 --sleep=10
```

#### Parsing the log file
Use the ```LogParser.py``` script to parse the log file and generate two CSV files to extract the planning and execution statistics. Example:
```
python3 generators/logging/LogParser.py BRSU_Floor0 1 --nRobots=5 --nExperiences=25
```

#### Analyse single experiment logs
Use the ```AnalyzeLogs.py``` script to generate plots from the planning and execution statistics of one experiement. Example:
```
python3 generators/logging/AnalyzeLogs.py BRSU_Floor0 1 --nRobots=5 --nExperiences=25
```

#### Analyze multiple experiments
Use the ```AnalyzeMultipleLogs.py``` script to generate plots that compare multiple test-cases of all the experiements. Example:
```
python3 generators/logging/AnalyzeMultipleLogs.py
```

## Sponsors
This project is supported by

* The <a href="http://semanticrobots.oru.se">Semantic Robots</a> Research Profile, funded by the <a href="http://www.kks.se/">Swedish Knowledge Foundation</a>
* The <a href="https://iliad-project.eu/">ILIAD Project</a>, funded by the <a href="https://ec.europa.eu/programmes/horizon2020/">EC H2020 Program</a>
* The iQMobility Project, funded by <a href="https://www.vinnova.se/">Vinnova</a>

## License
coordination_oru - Online coordination for multiple robots

Copyright &copy; 2017-2018 Federico Pecora

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
