clearvars
rosshutdown
rosinit

follower_message_prefix = "/tb3_0/"

size_x = 640;
size_y = 480;

%setup parameters
%sub = rossubscriber('/tb3_0/camera/parameter_updates');
%pause(1);



while(1)
    %setup ros subscriber
    sub = rossubscriber('/tb3_0/camera/rgb/image_raw');
    pause(1);

    msg1 = receive(sub,10);
    
    %convert from ros image type to matlab image type
    img = readImage(msg1);

    I_gs = convertRGBtoGS(img);
    I_bw = convertGStoBW(I_gs, 0.1);

    imwrite(I_bw , ['../MATLAB/',  'robot', '.jpg'],'jpg');

    new_I = imread("robot.jpg");
    
    offset = 20;

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
    %disp('process')
    
    %%Transformation to 3D coordinates
    
    %get raw depth information
    sub2 = rossubscriber('/tb3_0/camera/depth/image_raw');
    pause(1);
    
    msg2 = receive(sub2,10);
    
    %convert from ros image type to matlab image type
    depth_raw_image = readImage(msg2);
    
    %pick first loc point. bottom left square. Point of interest
    p_x = loc(1,1)
    p_y = loc(1,2)
    
    
    %get rgb camera information
    sub3 = rossubscriber('/tb3_0/camera/rgb/camera_info');
    pause(1);
    
    msg3 = receive(sub3,10);
     
    [X,Y,Z] = transform_image_to_3D(p_x, p_y, msg3, depth_raw_image)
     
    
     
     

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
