clearvars
rosshutdown
rosinit

%% ASSUMED COORDINATES
% From /tb3_0/base_link
% x [HEAD AXIS]
% ^
% |
% .--> y [SIDE AXIS]


%% Initialise variables

%coordinate frame of object of interest
head_val = 1;                   % X coord of object in camera coord frame. [head]
side_val = 0;                   % Y coord of object in camera coord frame. [turn]
turn_val = 0;                   % Z coord of object in camera coord frame. [height]

x = 1;                          % Orientation x 
y = 1;
z = 1;
w = 1;

%PID variables
kp_d = 0.5;                     % Proportional gain value.
ki_d = 0.05;                    % Integral gain value.
kd_d = 0.1;                     % Differential gain value. 

kp_a = 0.005;                   % Proportional gain value.
ki_a = 0.0003;                  % Integral gain value.
kd_a = 0.002;                   % Differential gain value. 

set_depth = 0.65;               % Following distance. Eg 0.65 m 
set_angle = 0;                  % Desired 0 angle difference.

linV_max = 0.26;                % linear velocity max speed.
linV_min = 0;

angV_max = 1.82;                % Right turn max speed. 
angV_min = -1.82;               % Left turn max speed. 

errorDepthBank = [0;0;0;0];     % Initialise bank data
errorAngleBank = [0;0;0;0];

flagA = 5;                      % Sets the size of the error banks



%% Main Procedure

while(1)
    
    % Subscribe to the ar_pose_markers topic and recieve the messages
    sub = rossubscriber('/ar_pose_marker');
   
    msg1 = receive(sub,10);
    
    % check if the ar tag marker exists in the message. If it does not exist
    % this iteration of the while loop is skipped and the follower robot
    % continues on previous trajectory. 
    if(msg1.Markers~=0)
        disp("Marker exists");
    else
        disp("Marker does not exist");
        continue; % skip the iteration fo the while loop
    end
    
    % assign PID controller object values
    head_val = msg1.Markers(1).Pose.Pose.Position.X; %X coord of object in camera coord frame.
    side_val = msg1.Markers(1).Pose.Pose.Position.Y; %Y coord of object in camera coord frame.
    turn_val = msg1.Markers(1).Pose.Pose.Position.Z; %Z coord of object in camera coord frame.
    
    %Calculate Error for depth and angle.    
    err_d = calculateErrorDepth(side_val,head_val,set_depth);
    [err_a,direction] = calculateErrorAngle(side_val,head_val);
    
    disp("distance = " + head_val);
    disp("side = " + side_val);
    disp("direction = " + direction);

    %Storing error values in a container.
    flagA = 5;
    errorDepthBank(flagA,:) = err_d;
    errorAngleBank(flagA,:) = err_a;
    flagA = flagA + 1;

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

function err_d = calculateErrorDepth(side_val,head_val,setDepth)
    coord = [0 0;side_val,head_val];
    eucDistance = pdist(coord,'euclidean');
    err_d = eucDistance - setDepth;
    if err_d < 0
        err_d = 0;
    end
end






