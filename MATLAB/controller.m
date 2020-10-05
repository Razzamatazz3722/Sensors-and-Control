% PID CONTROLLER
function controller()

%% ASSUMED COORDINATES
% x [HEAD AXIS]
% ^
% |
% .--> y [SIDE AXIS]

%% Initialise variables
%From image processing
head_val = 1;          %X coord of object in camera coord frame. [head]
side_val = 1;          %Y coord of object in camera coord frame. [turn]
turn_val = 1;          %Z coord of object in camera coord frame. [height]

x = 1;                 %Orientation x 
y = 1;
z = 1;
w = 1;

%PID variables
kp = 1;                 %Proportional gain value.
ki = 1;                 %Integral gain value.
kd = 1;                 %Differential gain value. 

set_depth = 1;          %Following distance. Ex. 1 meter or 1 unit. 
set_angle = 0;          %Desired 0 angle difference.

linV_max = 0.26;        %linear velocity max speed.
linV_min = 0;

angV_max = 1.82;        %Right turn max speed. 
angV_min = -1.82;       %Left turn max speed. 

                        %Initialise bank data
errorDepthBank = [0;0;0;0];
errorAngleBank = [0;0;0;0];


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
    out = P+I+D * direction;            %-1-> Turn right; 1-> Turn Left
    if out > maxOut
        out = maxOut;
    end
    
    if out < minOut
        out = minOut;
    end 
end

function [err_a,direction] = calculateErrorAngle(side_val,head_val)
    err_a = atan2d(side_val,head_val);
    if side_val > 0         %Object on Right     
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
end


while(1)   
%Calculate Error for depth and angle.    
err_d = calculateErrorDepth(side_val,head_val,set_depth);
[err_a,direction] = calculateErrorAngle(side_val,head_val);

%Storing error values in a container. 
flagA = 5;
errorDepthBank(flagA,:) = err_d;
errorAngleBank(flagA,:) = err_a;
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



