# Sensors-and-Control

Project Run and Installation instructions

Assuming that turtlebot and gazebo environment setup instructions from UTS online have been completed

1. git clone git@github.com:ros-perception/ar_track_alvar.git
   git clone git@github.com:Razzamatazz3722/Sensors-and-Control.git
   catkin_make
   
2. open Sensors_and_Control --> ar_tag_files (we will need to move some files) 
   the file "pr2_indiv.launch" will move to ar_track_alvar --> ar_track_alvar --> launch (and replace the file with the same name in this folder)
   the files "spawn_sdf.launch" and "turtlebot3_multi_ar.launch" will move to turtlebot_simulations --> turtlebot3_gazebo --> launch
   the folder "turtlebot3_arleader" should be moved to turtlebot_simulations --> turtlebot3_gazebo --> models
   
3. in the models folder --> turtlebot3_arleader, open model.sdf in a text editor
   
