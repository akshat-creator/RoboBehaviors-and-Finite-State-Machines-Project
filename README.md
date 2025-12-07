# RoboBehaviors-and-Finite-State-Machines-Project

**ENGR3590: Computation Introduction to Robotics &nbsp;|&nbsp; Fall 2025**  
**Akshat Jain &nbsp;|&nbsp; Satchel Schiavo &nbsp;|&nbsp; Owen Himsworth**

Goal in this project will be to program the Neato to execute a number of behaviors within a finite-state machine.
## Build & Run

```bash
# Build
colcon build --packages-select ros_behaviors_fsm
source install/setup.bash

# Run FSM (with params & RViz)
ros2 run ros_behaviors_fsm finite_state_controller

# View FSM state topic (live state)
ros2 topic echo /fsm/state
```
