clearvars
rosshutdown
rosinit

follower_message_prefix = "/tb3_0/"



%% Initialise variables
%From image processing
%X = 1;          %X coord of object in camera coord frame. [head]
%Y = 1;          %Y coord of object in camera coord frame. [turn]
%Z = 1;          %Z coord of object in camera coord frame. [height]
%depth_ri = 1;   %Depth raw image; depth from camera 
%P_x = 1;        %Selected point coord in the image frame. 
%P_y = 1;        %Selected point coord in the image frame. 
u = 1;          %New reading of x value of selected point coord.
v = 1;          %New reading of y value of selected point coord. 

size_x = 1392; %size of the image
size_y = 1024;

loc = []; %initialise blank array

%PID variables
kp = 1;                 %Proportional gain value.
ki = 1;                 %Integral gain value.
kd = 1;                 %Differential gain value. 

des_depth = 1;      %Following distance. Ex. 1 meter or 1 unit. 
des_angle = 0;      %Desired 0 angle difference.

linV_max = 0.26;    %linear velocity max speed.
linV_min = 0;

angV_max = 0.05;%1.82;    %Right turn max speed. 
angV_min = -0.05;%-1.82;   %Left turn max speed. 


%%
while(1)
    %setup ros subscriber
    sub = rossubscriber('/tb3_0/camera/rgb/image_raw');
    pause(1);

    msg1 = receive(sub,10);
    
    %convert from ros image type to matlab image type
    img = readImage(msg1);

    I_gs = convertRGBtoGS(img);
    I_bw = imadjust(I_gs, [0 0.1]);
    %I_bw = convertGStoBW(I_gs, 0.1);

    imwrite(I_bw , ['../MATLAB/',  'robot', '.jpg'],'jpg');

    new_I = imread("robot.jpg");
    
    offset = 20;
    
    if length(loc) > 0
        u = loc(1,1); %set u and v to the previous loc point
        v = loc(1,2);
    end;

    [msg,~,loc] = readBarcode(new_I,"QR-CODE");
    if length(loc) == 0
        disp("couldnt find barcode");
        imshow(I_bw);
        continue;
    end
    
    Imsg = insertShape(new_I, "FilledCircle", [loc, repmat(10, length(loc), 1)],"Color","red","Opacity",1);
    Imsg = insertShape(Imsg, "Line", [(size_x/2),0,(size_x/2),size_y], "Color", "blue", "LineWidth", 5);
    Imsg = insertShape(Imsg, "Line", [(size_x/2)+offset,0,(size_x/2)+offset,size_y], "Color", "green", "LineWidth", 2);
    Imsg = insertShape(Imsg, "Line", [(size_x/2)-offset,0,(size_x/2)-offset,size_y], "Color", "green", "LineWidth", 2);

    imshow(Imsg);
    
    %%Transformation to 3D coordinates
    
    %get raw depth information
    sub2 = rossubscriber('/tb3_0/camera/depth/image_raw');
    pause(1);
    
    msg2 = receive(sub2,10);
    
    %convert from ros image type to matlab image type
    depth_ri = readImage(msg2);
    
    %pick first loc point. bottom left square. Point of interest
    P_x = loc(1,1);
    P_y = loc(1,2);
    
    P_x = round(P_x);
    P_y = round(P_y);
    
    
    
    %get rgb camera information
%     sub3 = rossubscriber('/tb3_0/camera/rgb/camera_info');
%     pause(1);
%     
%     msg3 = receive(sub3,10);
%      
%     [X,Y,Z] = transform_image_to_3D(P_x, P_y, msg3, depth_ri);
%      
%     
%     %Initialise bank data
%     errorDepthBank = [0;0;0;0];
%     errorAngleBank = [0;0;0;0];
% 
%     %Calculate Error for depth and angle.    
%     err_d = calculateErrorDepth(X,Y,des_depth);
%     [err_aWorld,err_aImage,direction] = calculateErrorAngle(X,Y,u,v,P_x,P_y);
% 
%     %Storing error values in a container. 
%     flagA = 5;
%     errorDepthBank(flagA,:) = err_d;
%     errorAngleBank(flagA,:) = err_aImage;
%     flagA = flagA + 1;
% 
%     %Calculate PID Controller values
%     Pd = proportionalController(kp,errorDepthBank);
%     Pa = proportionalController(kp,errorAngleBank);
% 
%     Id = integralController(ki,errorDepthBank);
%     Ia = integralController(ki,errorAngleBank);
% 
%     Dd = differentialController(kd,errorDepthBank);
%     Da = differentialController(kd,errorAngleBank);
%     
%     disp("linear velocity")
%     out_d = calculateOutput(Pd,Id,Dd,linV_max,linV_min,1)
%     disp("angular velocity")
%     out_a = calculateOutput(Pa,Ia,Da,angV_max,angV_min,direction)
% 
%     %Publish to cmd/vel
%     cmd_msg = rosmessage('geometry_msgs/Twist');
%     %cmd_msg.Linear.X = out_d;
%     cmd_msg.Angular.Z = out_a;
% 
%     pubVel = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');
%     send(pubVel,cmd_msg);
     

%     %TEST ROTATING TO A POINT
%     %pick first loc point. bottom left square
%     p_x = loc(1,1)
%     p_y = loc(1,2)
%     
%     %setup ros message with 0 linear and angular
%     cmd_msg = rosmessage('geometry_msgs/Twist');
%     cmd_msg.Linear.X = 0;
%     cmd_msg.Linear.Y = 0;
%     cmd_msg.Linear.Z = 0;
%     cmd_msg.Angular.X = 0;
%     cmd_msg.Angular.Y = 0;
%     cmd_msg.Angular.Z = 0;
%     
%     offset = 20;
%     
%     if ((p_x < ((size_x/2)+offset)) & (p_x < ((size_x/2)-offset)))
%         cmd_msg.Angular.Z = 0.01;
%     elseif ((p_x > ((size_x/2)+offset) & (p_x > (size_x/2)-offset)))
%         cmd_msg.Angular.Z = -0.01;
%     else
%         cmd_msg.Angular.Z = 0;
%     end
%     
%     cmd_msg.Angular.Z
%     
%     %setup ros publisher
%     pub = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');
%     send(pub,cmd_msg);
%     
end


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

function [err_aWorld,err_aImage,direction] = calculateErrorAngle(X,Y,u,v,P_x,P_y)
    err_aWorld = atan2d(X,Y);
    err_aImage = atan2d((P_x-u),(P_y-v)); %Assuming x coord is not facing forward.
    if P_x - u > 0
        direction = 1;                    %Object is on the left.
        disp("object on left")
    else
        direction = -1;                   %Object is on the right.
        disp("object on right")
    end
    
end

function err_d = calculateErrorDepth(X,Y,setDepth)
    coord = [0 0;X,Y];
    eucDistance = pdist(coord,'euclidean');
    err_d = setDepth - eucDistance;
end
