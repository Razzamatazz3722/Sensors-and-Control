% PID CONTROLLER
function controller()

% clearvars
% rosshutdown
% rosinit

%% Initialise variables
%From image processing
X = 1;          %X coord of object in camera coord frame. [head]
Y = 1;          %Y coord of object in camera coord frame. [turn]
Z = 1;          %Z coord of object in camera coord frame. [height]
depth_ri = 1;   %Depth raw image; depth from camera 
P_x = 1;        %Selected point coord in the image frame. 
P_y = 1;        %Selected point coord in the image frame. 
u = 1;          %New reading of x value of selected point coord.
v = 1;          %New reading of y value of selected point coord. 

%PID variables
kp = 1;                 %Proportional gain value.
ki = 1;                 %Integral gain value.
kd = 1;                 %Differential gain value. 

des_depth = 1;      %Following distance. Ex. 1 meter or 1 unit. 
des_angle = 0;      %Desired 0 angle difference.

linV_max = 0.26;    %linear velocity max speed.
linV_min = 0;

angV_max = 1.82;    %Right turn max speed. 
angV_min = -1.82;   %Left turn max speed. 

%% FUNCTIONS
function P = proportionalController(kp,errorBank)
    round = size(errorBank);
    P = kp*errorBank(round(1));
end

function I = integralController(ki,errorBank)
    sumError = sum(errorBank);
    I = ki*sumError;
end

function D = differentialController(kd,errorBank)
    round = size(errorBank);
    slope = errorBank(round(1)) - errorBank(round(1)-1);
    D = kd*slope;
end

function out = calculateOutput(P,I,D,maxOut,minOut,direction)
    out = P+I+D * direction;
    if out > maxOut
        out = maxOut;
    end
    
    if out < minOut
        out = minOut;
    end 
end

function [err_aWorld,err_aImage,direction] = calculateErrorAngle(X,Y,P_x,P_y,u,v)
    err_aWorld = atan2d(X,Y);
    err_aImage = atan2d((u-P_x),(v-P_y)); %Assuming x coord is not facing forward.
    if u - P_x > 0
        direction = 1;                    %Object is on the right.
    else
        direction = -1;                   %Object is on the left. 
    end
    
end

function err_d = calculateErrorDepth(X,Y,setDepth)
    coord = [0 0;X,Y];
    eucDistance = pdist(coord,'euclidean');
    err_d = setDepth - eucDistance;
end


while(1)
%Initialise bank data
errorDepthBank = [0;0;0;0];
errorAngleBank = [0;0;0;0];
   
%Calculate Error for depth and angle.    
err_d = calculateErrorDepth(X,Y,des_depth);
[err_aWorld,err_aImage,direction] = calculateErrorAngle(X,Y,P_x,P_y,u,v);

%Storing error values in a container. 
flagA = 5;
errorDepthBank(flagA,:) = err_d;
errorAngleBank(flagA,:) = err_aImage;
flagA = flagA + 1;
    
%Calculate PID Controller values
Pd = proportionalController(kp,errorDepthBank);
Pa = proportionalController(kp,errorAngleBank);

Id = integralController(ki,errorDepthBank);
Ia = integralController(ki,errorAngleBank);

Dd = differentialController(kd,errorDepthBank);
Da = differentialController(kd,errorAngleBank);

out_d = calculateOutput(Pd,Id,Dd,linV_max,linV_min,1);

out_a = calculateOutput(Pa,Ia,Da,angV_max,angV_min,direction);

%Publish to cmd/vel
cmd_msg = rosmessage('geometry_msgs/Twist');
cmd_msg.Linear.X = out_d;
cmd_msg.Angular.Z = out_a;
 
pubVel = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');
send(pubVel,cmd_msg);
    
end
end



