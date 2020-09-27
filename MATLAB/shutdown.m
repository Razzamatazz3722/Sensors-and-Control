rosshutdown
rosinit

pub = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');

msg = rosmessage(pub);

msg.Linear.Y = 0;
msg.Linear.Z = 0;
msg.Angular.X = 0;
msg.Angular.Y = 0;
msg.Angular.Z = 0;
    
send(pub, msg);
pause(1);

rosshutdown