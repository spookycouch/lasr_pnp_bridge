# Example

An example can be found here: https://gitlab.com/sensible-robots/lasr_pnp_example

# Requirements

* PNP Node and JARP, available at https://github.com/iocchi/PetriNetPlans/tree/master/PNP
* Actionlib; Action Clients and Servers
* Robot Operating System

# Install

To install LASR PNP Bridge, clone this repository into the catkin workspace and `catkin_make` or build.

# Setup

**Plans**
 * Plans can be created using JARP (link above)
 * PNP can be seen as a series of actions in a flowchart
 * The flowchart is parsed by the PNP node to call actions implemented in C++

<br/>

**Places (PNP)**
 * LASR created a generic action that will send actionNames and params to any Action Server
 * This allows external Servers written in Python and C++ to provide implementations
 * Commands will be parsed and send a goal of strings - action, params[] - to a specified topic
 * Syntax for PNP place: `extern_topicName/ActionName_param1_param2_...`

<br/>

**Actions (Python and C++)**
 * Actions can be implemented in Python using Actionlib Simple Action Servers
 * Callbacks will receive data in the form of a goal of strings - action, params[] -
 * Implementation of actions using this information is then up to the developer
 * In the example, we use getattr() to call a method of name action

# Run plans using LASR PNP Bridge

**Terminals required:**
1. Roscore
2. PNP node
3. LASR PNP bridge
4. Action implementations (in the form of action servers)

I recommend creating a launch file to run the PNP node, LASR PNP bridge and
action servers simultaneously, where a "plans folder" can be specified under
PNP node.

<br/>

To run a plan, publish the plan name (- .pnml) of a plan located in the plans folder
to the ROS topic /planToExec. <br/> E.g. `rostopic pub /planToExec std_msgs/String "data: 'example_plan'"`

Any places in the PNP of the form `extern_topicName/actionName_param1_param2_...`
will be parsed by the LASR PNP Bridge, where the actionName and params will be sent
as a goal to an Action Server at topicName.

The specified Action Server can then exhibit the desired behaviour using information
passed by the LASR PNP Bridge.

# Flowchart

![](./docs/pnp_flow.png)

# Additional Features

**Result: exit_status**
 - An int that can be used to indicate the exit status of an action


**Result: condition_event**
 - Any strings returned as a result will be relayed by LASR PNP Bridge as PNP 
   condition events (i.e. truth statements)
 

**Result: set_variables**
 - PNP variables can be set using set_variable, where a VariableValue type is employed
 - variable refers to the variable name to be updated
 - value refers to the new value the variable will be updated to

**Topic: /PNPConditionIn**
 - A subscriber from LASR PNP Bridge to the topic `/PNPConditionIn` relays any strings
   as PNP condition events (i.e. truth statements)