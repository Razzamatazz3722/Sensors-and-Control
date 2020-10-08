clearvars
rosshutdown
rosinit
%pyenv('Version','/usr/local/bin/python3.9');
%%Initialise Variables
%/home/rachael/catkin_ws/src/ar_track_alvar/ar_track_alvar_msgs

addpath('/home/rachael/git/ar_track_alvar/matlab_gen/msggen')
savepath

folderpath = "/home/rachael/catkin_ws/src/ar_track_alvar/";
rosgenmsg(folderpath)

sub = rossubscriber('/ar_pose_marker');
pause(1);

msg1 = receive(sub,10)

%%Subscribe to topics

