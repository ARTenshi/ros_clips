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
/clips_ros/clipspy_assert_command
```

of type `clips_ros/PlanningCmdClips`

where PlanningCmdClips.msg:

```
string name
string params
int32 id
int32 successful
```

2. 
```
/clips_ros/clipspy_eval_command
```

of type `clips_ros/PlanningCmdSend`

where PlanningCmdSend.msg:

```
string command
```


3. 
```
/clips_ros/clipspy_assert_string
```

of type `std_msgs/String`

4. 
```
/clips_ros/clipspy_eval_string
```

of type `std_msgs/String`



5. 
```
/clips_ros/clipspy_run
```

of type `std_msgs/Int64`

6. 
```
/clips_ros/clipspy_reset
```

of type `std_msgs/Empty`


7. 
```
/clips_ros/clipspy_clear
```

of type `std_msgs/Empty`

8. 
```
/clips_ros/clipspy_print_facts
```

of type `std_msgs/Empty`

9. 
```
/clips_ros/clipspy_print_rules
```

of type `std_msgs/Empty`

10. 
```
/clips_ros/clipspy_print_templates
```

of type `std_msgs/Empty`

11. 
```
/clips_ros/clipspy_print_instances
```

of type `std_msgs/Empty`

12. 
```
/clips_ros/clipspy_print_agenda
```

of type `std_msgs/Empty`



13. 
```
/clips_ros/clipspy_send_command
```

of type `std_msgs/String`

14. 
```
/clips_ros/clipspy_send_and_run_command
```

of type `std_msgs/String`

15. 
```
/clips_ros/clipspy_load_file
```

of type `std_msgs/String`


**Services**

TBD

### Initialization

Start `ROS` in one terminal:

```
roscore
```

Start the ros_clips in a different terminal:

```
source ~/ros_clips_ws/devel/setup.bash
rosrun clips_ros ros_clipspy_node.py
```

### Usage

**Cubes example**

First, load the clp file:

```
source ~/ros_clips_ws/devel/setup.bash
rostopic pub /clips_ros/clipspy_load_file std_msgs/String "data: '~/ros_clips_ws/src/ros_clips/data/clp/cubes_stacks.clp'" 
```

Then, check the current facts:

```
rostopic pub /clips_ros/clipspy_print_facts std_msgs/Empty "{}"
```

and rules:

```
rostopic pub /clips_ros/clipspy_print_rules std_msgs/Empty "{}"
```

Also, you can run a plan one stepa at a time:

```
rostopic pub /clips_ros/clipspy_run std_msgs/Int64 "data: 1"
```

or all at once:

```
rostopic pub /clips_ros/clipspy_run std_msgs/Int64 "data: 0"
```
