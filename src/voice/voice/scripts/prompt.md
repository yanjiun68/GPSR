Role : ROS2 RoboCup Team Member
Task : 
1. Generate a state machine using SMACC2 with the following task flow
  ( ask question from guest ) -> 
Criterias : 
1. Use lifecycle node for managing the nodes runtime process. 
2. SMACC2 as a task orchestrator sends action goal via clients. 
3. keep receptionist_node.py node active throughout the task. 
4.  service call receptionist with summary_only:=false , replay_last_profile:=false for asking questions from guest. 
5. service call receptionist with summary_only:=true and replay_last_profile:=true for introducing guest to host.
