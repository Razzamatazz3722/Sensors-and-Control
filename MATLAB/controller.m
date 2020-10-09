% PID CONTROLLER
function controller()


%% NOTES:
% Testing with Ar_track alvar: 
% -> Assign subscribe values to head/side/turn_val. With X is head, y is side, z is height. 
%This is from the base_link coordinate frame. 

% -> Tune the PID K values. I have found slowing the angle turn works the
% best. Therefore, have those values (kp_a etc ...) really small. If the
% runs in circles then that means angle values are too high. Set them
% lower.

%-> Build up the controller with just the proportianal then continue adding
% derivative and integral controller. I suggest PD then PID. IF working
% with just the proportial controller remember that 'out' function needs I and D
% values aswell thereby you need to set those values to zero. 

% -> When tunning, the higher the number the faster the faster the output
% will be come. Especially in the proportianal. For others, make them
% larger if you want more of that controller to have a larger influence
% with the output. 

%-> Proportional [Present] -> To approach the target value. 
%-> Integral [Past] -> To accelerate the approach value. This can be
%replaced with a higher proportial k value. 
%-> Derivative [Future] -> Acts as a brake. The bigger the slope the 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 
% clearvars
% rosshutdown
% rosinit
% 
% follower_message_prefix = "/tb3_0/"


%% ASSUMED COORDINATES
% From /tb3_0/base_link
% x [HEAD AXIS]
% ^
% |
% .--> y [SIDE AXIS]

%% Initialise variables
%From image processing
head_val = 1;          %X coord of object in camera coord frame. [head]
side_val = 0;          %Y coord of object in camera coord frame. [turn]
turn_val = 0;          %Z coord of object in camera coord frame. [height]

x = 1;                 %Orientation x 
y = 1;
z = 1;
w = 1;

%% TUNING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PID variables
kp_d = 0.5;                 %Proportional gain value.
ki_d = 0.05;                %Integral gain value.
kd_d = 0.1;                 %Differential gain value. 

kp_a = 0.005;                 %Proportional gain value.
ki_a = 0.0003;                %Integral gain value.
kd_a = 0.002;                 %Differential gain value. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% SET VALUES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set_depth = 0.75;           %Following distance. Ex. 1 meter or 1 unit. I have set it to 0.75 as greater than 1 we may have issues detecting AR Tag. 
set_angle = 0;              %Desired 0 angle difference.

linV_max = 0.26;            %linear velocity max speed.
linV_min = 0;

angV_max = 1.82;            %Right turn max speed. 
angV_min = -1.82;           %Left turn max speed. 

                            %Initialise bank data
errorDepthBank = [0;0;0;0];
errorAngleBank = [0;0;0;0];

flagA = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    if err_a < 0
        err_a = 0;
    end
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


while(1)   
%Calculate Error for depth and angle.    
err_d = calculateErrorDepth(side_val,head_val,set_depth);
[err_a,direction] = calculateErrorAngle(side_val,head_val);

%Storing error values in a container. 
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
 
pubVel = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');
send(pubVel,cmd_msg);    
end
end



