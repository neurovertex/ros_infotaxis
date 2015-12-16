ROS Infotaxis
=============

This node controls a robot to look for a gas source, using the infotaxis algorithm. For detail about infotaxis itself, see [neurovertex/infotaxis](https://github.com/neurovertex/infotaxis)

Requirements
============
Software
--------

This node needs a map and a location. The two obvious approaches are map_server + AMCL or SLAM. This node will also need the navigation stack (move_base), and won't run without a global costmap and a local costmap being published.

Hardware
--------

*If* not in simulation (which is so far the only implemented mode, actual sensors reading coming soon), you'll need a gas sensor mounted on a mobile robot. That's about it. Format of the sensor readings is TBD yet.

Implementation details
======================

The infotaxis algorithm is obviously not implementable "as is" on a real robot. Special measures have to be taken to account for obstacles, the continuous nature of the robot's position, moving times, etc... So here are the main differences between the algorithm and this implementation.

The node is implemented as a finite-state automaton. States are DETECTING, PLANNING, MOVING and EXIT. Entry state is DETECTING and exit state is ... obvious enough.

Detecting
---------
When reaching this state, the robot will wait for *dt* seconds while reading from the sensor as to get a number of detections (as per the algorithm). How this is done with a real sensor is yet TBD. In Simulation mode, the detections will be randomly generated according to a Poisson distribution. Since the robot spends much more time moving between goals than arrived, most of the detections are done during the moving state. Then the sate is set to PLANNING.

Planning
--------
Here the nodes determines where to go next. It generates a number of poses around its current location, then picks the one with the highest expected entropy reduction, and sends it to the navigation stack. Since calculating said values takes some time, most of the planning is actually done during the moving state.

### Next steps generation
Every step in the MOVING state, *nextsteps* new goals are generated. They are spread around the current destination by steps of 360/*nextsteps*° (i.e 90° for *nextsteps*=4). The distance is random, and its min and max possible values vary depending on on the "interest" of the area the destination is in, the interest is calculated by taking the N~th root of the average value of the grid around the center (N ~ 25). The actual boundaries for the distance are then calculated from the parameters *min_stepdist*, *med_stepdist*, *max_stepdist*; the lower and upper bounds are scaled respectively between [min,med] and [med,max] along the value of the interest. That way, the robot moves makes larger steps in low-interest areas.

Moving
------
This states is composed of several phases. First, we check on the current navigation goal.

- If it is reached, we set the next state to DETECTING.
- If it failed to reach it, we set the next state to PLANNING.
- If it is still going, we generate *nextsteps* new goals around the expected destination.

Then, if *detect_while_moving* is set to true, the robot will also run the DETECTING state.

Output
======

Images
------
If *print_images* is set to true, this node will output an image to the *image_dir* folder describing the current state of the probability field after every iteration in the DETECTING state. If *detecting_while_moving* is set to true, it will also output an image every *moving_image_write_frequency* detections while moving (as to avoid outputting several thousand frames). If *simulate_source* is true, the mean concentration field will be written as well, to *image_dir*/meanfield.png

These images can be compiled into a gif with the makegif.sh from [neurovertex/infotaxis](https://github.com/neurovertex/infotaxis) (though it requires *image_dir* to be "pictures").

![example](http://i.imgur.com/ONfJRu1.gif)

Building
========
Requirements:
- Compiler with C++11 support
- libpng and png++
- Catkin, obviously

Clone this repository in a catkin workspace and create a "lib" directory inside. Clone  [neurovertex/infotaxis](https://github.com/neurovertex/infotaxis) somewhere, build it, then copy/move/symlink libinfotaxis.a to ros_infotaxis/lib/. From there, `catkin build` (or `catkin_make`) should work.

Parameters
==========

TODO
