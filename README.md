# ros_clips

ROS interface for the C Language Integrated Production System or [CLIPS](https://clipsrules.net/)

This repo uses [clipspy](https://github.com/noxdafox/clipspy/) for CLIPS on Python 3.

## Setup

1. Create an env of your choice.

e.g.

First, create a workspace:

```
cd ~
mkdir -p ros_clips_ws/src
```

Then, clone this repository into the src folder:

```
cd ~/ros_clips_ws/src
git clone https://github.com/ARTenshi/ros_clips.git
```

2. Run ```pip install -r requirements.txt```

3. Build the project:

e.g 

```
cd ~/ros_clips_ws
catkin_make
```

## ROS-CLIPS Bridge

### Structure

**Topics**


1. 
```
/clips_ros/clipspy_reset
```

of type `std_msgs/Empty`


2. 
```
/clips_ros/clipspy_clear
```

of type `std_msgs/Empty`


3. 
```
/clips_ros/clipspy_load_file
```

of type `std_msgs/String`


3. 
```
clips_ros/clipspy_build_rule
```

of type `std_msgs/String`


3. 
```
/clips_ros/clipspy_build_template
```

of type `std_msgs/String`


3. 
```
/clips_ros/clipspy_build_facts
```

of type `std_msgs/String`


3. 
```
/clips_ros/clipspy_load_fact
```

of type `std_msgs/String`


**Services**

1. 
```
/clips_ros/run_planning
```

of type `clips_ros/RunPlanning` 


where RunPlanning.srv:

```
int64 steps
---
StringArray plan
```

2. 

```
/clips_ros/get_facts
```

of type `clips_ros/GetPrintMessage` 

and

```
/clips_ros/get_rules
```

of type `clips_ros/GetPrintMessage` 


where GetPrintMessage.srv:

```
---
StringArray data
```

### Initialization

Start `ROS` in one terminal:

```
roscore
```

Start the ros_clips in a different terminal:

```
source ~/ros_clips_ws/devel/setup.bash
rosrun clips_ros ros_clipspy_bridge.py
```

### Usage

**Cubes example**

First, load the clp file:

```
source ~/ros_clips_ws/devel/setup.bash
rostopic pub /clips_ros/clipspy_load_file std_msgs/String "data: '~/ros_clips_ws/src/ros_clips/data/clp/cubes_stacks_rules.clp'" 
```

Then, check the current rules:

```
rosservice call /clips_ros/get_rules "{}"
```

and facts:

```
rosservice call /clips_ros/get_facts "{}"
```

You can add some new facts as follows:

```
rostopic pub /clips_ros/clipspy_load_fact std_msgs/String "data: '(get-initial-state stacks 2)'"
rostopic pub /clips_ros/clipspy_load_fact std_msgs/String "data: '(stack A B C)'"
rostopic pub /clips_ros/clipspy_load_fact std_msgs/String "data: '(stack D E F)'"
```

You can also add a new goal from a template. First, check the templates:

```
rosservice call /clips_ros/get_templates "{}"
```

Then, add the new goal as a fact:

```
rostopic pub /clips_ros/clipspy_load_fact std_msgs/String "data: '(goal (move F)(on-top-of A))'"
```

Finally, you can get a plan one step a at a time:

```
rosservice call /clips_ros/run_planning "steps: 1"
```

or all at once:

```
rosservice call /clips_ros/run_planning "steps: 0"
```

You can check the current facts, after executing the full plan:

```
rosservice call /clips_ros/get_facts "{}"
```

add a new goal:

```
rostopic pub /clips_ros/clipspy_load_fact std_msgs/String "data: '(goal (move A)(on-top-of F))'"
```

and get the new plan:

```
rosservice call /clips_ros/run_planning "steps: 0"
```
