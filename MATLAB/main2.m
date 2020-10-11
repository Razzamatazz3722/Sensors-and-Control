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
% Distance Gain Values
kp_d = 0.5;                     % Proportional gain value.
ki_d = 0.05;                    % Integral gain value.
kd_d = 0.1;                     % Differential gain value. 

% Angular Gain Values
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

    %Calculate PID Controller values for depth and angle
    Pd = proportionalController(kp_d,errorDepthBank); %depth proportional controller value
    Pa = proportionalController(kp_a,errorAngleBank); %angular proportional controller value

    Id = integralController(ki_d,errorDepthBank); %depth integral controller value
    Ia = integralController(ki_a,errorAngleBank); %angular integral controller value

    Dd = differentialController(kd_d,errorDepthBank); %depth differential controller value
    Da = differentialController(kd_a,errorAngleBank); %angular differential controller value
    
    out_d = calculateOutput(Pd,Id,Dd,linV_max,linV_min,1); %linear velocity output value
    out_a = calculateOutput(Pa,Ia,Da,angV_max,angV_min,direction); %angular velocity output value

    %Create rosmessage and publish to cmd/vel
    cmd_msg = rosmessage('geometry_msgs/Twist');
    cmd_msg.Linear.X = out_d;
    cmd_msg.Angular.Z = out_a;
    disp(out_d + " " + out_a);

    pubVel = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');
    send(pubVel,cmd_msg);    

end 


%% FUNCTIONS

% Name: proportionalController
% Inputs: kp - proportional gain value, errorBank - matrix of accumulated 
% errors
% Outputs: P - proportional controller value
% Description: Calculates the proportional value of the controller. NOTE:
% it is assumed that latest error is held in the last/bottom position in the
% matrix. 
function P = proportionalController(kp,errorBank)
    increment_err = size(errorBank);
    P = kp*errorBank(increment_err(1));
end

% Name: integralController
% Inputs: ki - integral gain value, errorBank - matrix of accumulated
% errors
% Outputs: I - integral controller value
% Description: Calculates the integral value of the controller
function I = integralController(ki,errorBank)
    sumError = sum(errorBank);
    I = ki*sumError;
end

% Name: differentialController
% Inputs: kd - differential gain value, errorBank - matrix of acucmulated
% errors
% Outputs: D - derivatice controller value
% Description: Calculates the derivatice value of the controller
function D = differentialController(kd,errorBank)
    increment_err = size(errorBank);
    slope = errorBank(increment_err(1)) - errorBank(increment_err(1)-1);
    D = kd*slope;
end

% Name: calculateOutput
% Inputs: P - proportional controller value, I - integral controller value,
% D - differential controller value, maxOut - maximum output value, minOut
% - minimum output value, direction - sets the polarity of the output
% Outputs: out - output value to control the robot
% Description: calculates the output value based on the PID values and the
% minimum and maximum output values. 
function out = calculateOutput(P,I,D,maxOut,minOut,direction)
    out = P+I+D * direction;           %-1-> Turn right; 1-> Turn Left
    disp("P=" + P + "  I=" + I + "  D=" + D ); 
    if out > maxOut
        out = maxOut;
    end
    
    if out < minOut
        out = minOut;
    end 
end

% Name: calculateErrorAngle
% Inputs: side_val - the value of the side coordinate of the object of
% interest, head_val - the value of the head coordinate of the object of 
% interest. 
% Outputs: err_a - angle error, direction - positive if the point is on the 
% left of the centre of the follower robot, negative if the point is on the 
% right of the centre of the follower robot
% Description: Calculates the current angle error to the point of interest
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

% Name: calculateErrorDepth
% Inputs: side_val - the value of the side coordinate of the object of
% interest, head_val - the value of the head coordinate of the object of 
% interest. 
% Outputs: err_d - depth error
% Description: calculates the depth error between the robots current
% position and the point of interest
function err_d = calculateErrorDepth(side_val,head_val,setDepth)
    coord = [0 0;side_val,head_val];
    eucDistance = pdist(coord,'euclidean');
    err_d = eucDistance - setDepth;
    if err_d < 0
        err_d = 0;
    end
end






