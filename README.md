# Sensors-and-Control

Project Run and Installation instructions

For Ubunti 18.04
ROS Melodic 

Assuming that turtlebot and gazebo environment setup instructions from UTS online have been completed
on Matlab have the ROS toolbox installed and ROS Toolbox interface for ROS Custom Messages  toolbox installed


1. git clone git@github.com:ros-perception/ar_track_alvar.git

   git clone git@github.com:Razzamatazz3722/Sensors-and-Control.git
   
   catkin_make
   
   source devel/setup.bash
   
2. Open 'Sensors-and-Control' folder --> ar_tag_files (we will need to move some files) 

   2.1)
   Look for the file "turtlebot3_teleop_key.launch" and move to turtlebot3 (the ROS package in catkin_ws) --> turtlebot3_teleop --> launch (replace file inside      folder).
   
   2.2)
   Look for the file "pr2_indiv.launch" will move to ar_track_alvar (the ROS package in catkin_ws) --> ar_track_alvar --> launch (and replace the file with the      same name in this folder).
   
   2.3)
   Look for the files "spawn_sdf.launch" and "turtlebot3_multi_ar.launch" will move to turtlebot_simulations (the ROS package in catkin_ws) --> turtlebot3_gazebo    --> launch
   
   2.4)
   Look for the folder "turtlebot3_arleader" should be moved to turtlebot_simulations (the ROS package in catkin_ws) --> turtlebot3_gazebo --> models
   
3. In the models folder --> turtlebot3_arleader, open model.sdf in a text editor then
   'ctrl+f' and search for "cherise" change all 5 iterations to the name of your user/home directory name and save 

4. In a terminal: 

   roslaunch turtlebot3_gazebo turtlebot3_multi_ar.launch

   roslaunch ar_track_alvar pr2_indiv.launch

   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 

   matlab (open matlab).
                  
5. Ensure folder Sensors-and-Control is added to the path and open main2.m

6. In the matlab command line: folderpath = "/home/YOURNAME/catkin_ws/src/ar_track_alvar/"
                               rosgenmsg(folderpath)
                               follow instructions prompted in command line 
                               if permissions error occurs, type sudo chmod 777 pathdef.m  into your terminal and enter your password when prompted 
7. Run the Matlab code
8. Teleop the leader around VERY SLOWLY please
9. If you encounter errors please google it, don't ask us thanks (maleen is real mvp wo knows all)  :)

PERCENTAGE CONTRIBUTIONS

Rachael 33.33%

Rebo 33.33%

Cherise 33.33%

