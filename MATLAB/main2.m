clearvars
rosshutdown
rosinit

%% Initialise variables

%From image processing
head_val = 1;          %X coord of object in camera coord frame. [head]
side_val = 0;          %Y coord of object in camera coord frame. [turn]
turn_val = 0;          %Z coord of object in camera coord frame. [height]

x = 1;                 %Orientation x 
y = 1;
z = 1;
w = 1;

%PID variables
kp_d = 0.5;                 %Proportional gain value.
ki_d = 0.05;                 %Integral gain value.
kd_d = 0.1;                 %Differential gain value. 

kp_a = 0.005;                 %Proportional gain value.
ki_a = 0.0003;                 %Integral gain value.
kd_a = 0.002;                 %Differential gain value. 

set_depth = 0.65;          %Following distance. Ex. 1 meter or 1 unit. 
set_angle = 0;          %Desired 0 angle difference.

linV_max = 0.26;        %linear velocity max speed.
linV_min = 0;

angV_max = 1.82;        %Right turn max speed. 
angV_min = -1.82;       %Left turn max speed. 

                        %Initialise bank data
errorDepthBank = [0;0;0;0];
errorAngleBank = [0;0;0;0];

errorDepthBank2 = {0,0,0,0,0};
errorAngleBank2 = {0,0,0,0,0};

flagA = 5;



%% Main Procedure

while(1)
    %Subscribe to the ar_pose_markers topic and recieve the messages
    sub = rossubscriber('/ar_pose_marker');
    %pause(0.3);
   
    msg1 = receive(sub,10);
    
    %check if the ar tag marker exists in the message. If it does not exist
    %this iteration of the while loop is skipped. 
    if(msg1.Markers~=0)
        disp("Marker exists");
    else
        disp("Marker does not exist");
        continue; %skip the iteration fo the while loop
    end
    
    %assign PID controller object values
    head_val = msg1.Markers(1).Pose.Pose.Position.X; %X coord of object in camera coord frame.
    side_val = msg1.Markers(1).Pose.Pose.Position.Y; %Y coord of object in camera coord frame.
    turn_val = msg1.Markers(1).Pose.Pose.Position.Z; %Z coord of object in camera coord frame.
    
    disp("distance = " + head_val);
    disp("side = " + side_val);
    
%     obj_z = head_val;
%     obj_y = side_val;
%     obj_x = turn_val;x
%     disp("x =" + obj_x + " y =" + obj_y + " z = " + obj_z);
% 
%     angular_offset = 0.02;
%     linear_offset = 0.1;
% 
%     depth_goal = 0.7;
% ;
%     cmd_msg = rosmessage('geometry_msgs/Twist');
%     cmd_msg.Linear.X = 0;
%     cmd_msg.Linear.Y = 0;
%     cmd_msg.Linear.Z = 0;
%     cmd_msg.Angular.X = 0;
%     cmd_msg.Angular.Y = 0;
%     cmd_msg.Angular.Z = 0;
% 
% 
%     if ((obj_x < 0+angular_offset) & (obj_x < 0-angular_offset))
%         cmd_msg.Angular.Z = 0.02;
%         disp("rotate left");
%     elseif ((obj_x > 0+angular_offset) & (obj_x > 0-angular_offset))
%         cmd_msg.Angular.Z = -0.02;
%         disp("rotate right");
%     else
%         cmd_msg.Angular.Z = 0;
%         disp("rotate off");
%     end
%     
%     %set linear velocity
%     if((obj_z > depth_goal+linear_offset) & (obj_z > depth_goal-linear_offset))
%         cmd_msg.Linear.Z = 0.5;
%         disp("go forward");
%     elseif((obj_z < depth_goal+linear_offset) & (obj_z < depth_goal-linear_offset))
%         cmd_msg.Linear.Z = -0.3;
%         disp("go backwards");
%     else
%         cmd_msg.Linear.Z = 0;
%         disp("stop");
%     end
%     
%     pub = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');
%     send(pub,cmd_msg);
    
    
    %Calculate Error for depth and angle.    
    err_d = calculateErrorDepth(side_val,head_val,set_depth);
    [err_a,direction] = calculateErrorAngle(side_val,head_val);
    
    disp("direction = " + direction);

    %Storing error values in a container.
    flagA = 5;
    errorDepthBank(flagA,:) = err_d;
    errorAngleBank(flagA,:) = err_a;
    flagA = flagA + 1;
    
    %disp(errorDepthBank2);
    %errorDepthBank2 = errorDepthBank2(2:end);
    %errorDepthBank2{end+1} = err_d;
    %disp(errorDepthBank2);
    
    
   

    %Calculate PID Controller values
    Pd = proportionalController(kp_d,errorDepthBank);
    Pa = proportionalController(kp_a,errorAngleBank);

    Id = integralController(ki_d,errorDepthBank);
    Ia = integralController(ki_a,errorAngleBank);

    Dd = differentialController(kd_d,errorDepthBank);
    Da = differentialController(kd_a,errorAngleBank);

    out_d = calculateOutput(Pd,Id,Dd,linV_max,linV_min,1);
    out_a = calculateOutput(Pa,Ia,Da,angV_max,angV_min,direction);

    %Publish to cmd/vel
    cmd_msg = rosmessage('geometry_msgs/Twist');
    cmd_msg.Linear.X = out_d;
    cmd_msg.Angular.Z = out_a;
    disp(out_d + " " + out_a);

    pubVel = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');
    send(pubVel,cmd_msg);    

end


%% FUNCTIONS
function P = proportionalController(kp,errorBank)
    increment_err = size(errorBank);
    P = kp*errorBank(increment_err(1));
    %tmp = errorBank{5};
    %P = kp*tmp;
end

function I = integralController(ki,errorBank)
    sumError = sum(errorBank);
    I = ki*sumError;
end

function D = differentialController(kd,errorBank)
    increment_err = size(errorBank);
    slope = errorBank(increment_err(1)) - errorBank(increment_err(1)-1);
    D = kd*slope;
end

function out = calculateOutput(P,I,D,maxOut,minOut,direction)
    out = P+I+D * direction;           %-1-> Turn right; 1-> Turn Left
    disp("P=" + P + "  I=" + I + "  D=" + D ); 
    %out = P*direction;
    if out > maxOut
        out = maxOut;
    end
    
    if out < minOut
        out = minOut;
    end 
end

function [err_a,direction] = calculateErrorAngle(side_val,head_val)
    err_a = atan2d(side_val,head_val);
    %if err_a < 0
    %    err_a = 0;
    %end
    if side_val < 0         %Object on Right     
        direction = -1;    
    else                    %Object on Left
        direction =  1;   
    end    
end

%     function [err_a,direction] = calErrAngle_ar(z_val)
%         err_a = z_val;        
%         if err_a > 0        %Object on Right
%             direction = -1;
%         else                %Object on Left
%             direction = 1;
%         end
%     end



function err_d = calculateErrorDepth(side_val,head_val,setDepth)
    coord = [0 0;side_val,head_val];
    eucDistance = pdist(coord,'euclidean');
    err_d = eucDistance - setDepth;
    if err_d < 0
        err_d = 0;
    end
end









% obj_x = head_val;
%     obj_y = side_val;
%     obj_z = turn_val;
%     disp("x =" + obj_x + " y =" + obj_y + " z = " + obj_z);
%     
%     angular_offset = 0.02;
%     linear_offset = 0.1;
%     
%     depth_goal = 0.7;
%     
%     cmd_msg = rosmessage('geometry_msgs/Twist');
%     cmd_msg.Linear.X = 0;
%     cmd_msg.Linear.Y = 0;
%     cmd_msg.Linear.Z = 0;
%     cmd_msg.Angular.X = 0;
%     cmd_msg.Angular.Y = 0;
%     cmd_msg.Angular.Z = 0;
% 
% 
%     % set angular v obj_x = head_val;
%     obj_y = side_val;
%     obj_z = turn_val;
%     disp("x =" + obj_x + " y =" + obj_y + " z = " + obj_z);
%     
%     angular_offset = 0.02;
%     linear_offset = 0.1;
%     
%     depth_goal = 0.7;
%     
%     cmd_msg = rosmessage('geometry_msgs/Twist');
%     cmd_msg.Linear.X = 0;
%     cmd_msg.Linear.Y = 0;
%     cmd_msg.Linear.Z = 0;
%     cmd_msg.Angular.X = 0;
%     cmd_msg.Angular.Y = 0;
%     cmd_msg.Angular.Z = 0;
% 
% 
%     % set angular velocity
%     if ((obj_x < 0+angular_offset) & (obj_x < 0-angular_offset))
%         cmd_msg.Angular.Z = 0.02;
%         disp("rotate left");
%     elseif ((obj_x > 0+angular_offset) & (obj_x > 0-angular_offset))
%         cmd_msg.Angular.Z = -0.02;
%         disp("rotate right");
%     else
%         cmd_msg.Angular.Z = 0;
%         disp("rotate off");
%     end
%     
%     %set linear velocity
%     if((obj_z > depth_goal+linear_offset) & (obj_z > depth_goal-angular_offset))
%         cmd_msg.Linear.Z = 0.5;
%         disp("go forward");
%     elseif((obj_z < depth_goal+linear_offset) & (obj_z < depth_goal-angular_offset))
%         cmd_msg.Linear.Z = -0.3;
%         disp("go backwards");
%     else
%         cmd_msg.Linear.Z = 0;
%         disp("stop");
%     end
    
    %pub = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');
    %send(pub,cmd_msg);elocity
%     if ((obj_x < 0+angular_offset) & (obj_x < 0-angular_offset))
%         cmd_msg.Angular.Z = 0.02;
%         disp("rotate left");
%     elseif ((obj_x > 0+angular_offset) & (obj_x > 0-angular_offset))
%         cmd_msg.Angular.Z = -0.02;
%         disp("rotate right");
%     else
%         cmd_msg.Angular.Z = 0;
%         disp("rotate off");
%     end
%     
%     %set linear velocity
%     if((obj_z > depth_goal+linear_offset) & (obj_z > depth_goal-angular_offset))
%         cmd_msg.Linear.Z = 0.5;
%         disp("go forward");
%     elseif((obj_z < depth_goal+linear_offset) & (obj_z < depth_goal-angular_offset))
%         cmd_msg.Linear.Z = -0.3;
%         disp("go backwards");
%     else
%         cmd_msg.Linear.Z = 0;
%         disp("stop");
%     end
    
    %pub = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');
    %send(pub,cmd_msg);