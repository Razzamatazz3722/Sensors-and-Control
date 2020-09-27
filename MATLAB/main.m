rosshutdown
rosinit

follower_message_prefix = "/tb3_0/"

%setup parameters
%sub = rossubscriber('/tb3_0/camera/parameter_updates');
%pause(1);



while(1)
    %setup ros subscriber
    sub = rossubscriber('/tb3_0/camera/rgb/image_raw');
    pause(1);

    msg2 = receive(sub,10);
    
    %convert from ros image type to matlab image type
    img = readImage(msg2);

    I_gs = convertRGBtoGS(img);
    I_bw = convertGStoBW(I_gs, 0.1);

    imwrite(I_bw , ['../MATLAB/',  'robot', '.jpg'],'jpg');

    new_I = imread("robot.jpg");
    
    offset = 20;

    [msg,~,loc] = readBarcode(new_I,"QR-CODE");
    if length(loc) == 0
        disp("couldnt find barcode");
        continue;
    end
    
    Imsg = insertShape(new_I, "FilledCircle", [loc, repmat(10, length(loc), 1)],"Color","red","Opacity",1);
    Imsg = insertShape(Imsg, "Line", [(1920/2),0,(1920/2),1080], "Color", "blue", "LineWidth", 5);
    Imsg = insertShape(Imsg, "Line", [(1920/2)+offset,0,(1920/2)+offset,1080], "Color", "green", "LineWidth", 2);
    Imsg = insertShape(Imsg, "Line", [(1920/2)-offset,0,(1920/2)-offset,1080], "Color", "green", "LineWidth", 2);

    imshow(Imsg);
    %disp('process')

    %TEST ROTATING TO A POINT
    %pick first loc point. bottom left square
    p_x = loc(1,1)
    p_y = loc(1,2)
    
    %setup ros message with 0 linear and angular
    cmd_msg = rosmessage('geometry_msgs/Twist');
    cmd_msg.Linear.X = 0;
    cmd_msg.Linear.Y = 0;
    cmd_msg.Linear.Z = 0;
    cmd_msg.Angular.X = 0;
    cmd_msg.Angular.Y = 0;
    cmd_msg.Angular.Z = 0;
    
    offset = 20;
    
    if ((p_x < ((1920/2)+offset)) & (p_x < ((1920/2)-offset)))
        cmd_msg.Angular.Z = 0.01;
    elseif ((p_x > ((1920/2)+offset) & (p_x > (1920/2)-offset)))
        cmd_msg.Angular.Z = -0.01;
    else
        cmd_msg.Angular.Z = 0;
    end
    
    cmd_msg.Angular.Z
    
    %setup ros publisher
    pub = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');
    send(pub,cmd_msg);
    
    
    %msg = rosmessage(pub);
    %msg.Linear.X = 0;
   % xyText =  loc(1,:);
    %Imsg = insertText(img,xyText,msg,"BoxOpacity",1,"FontSize",25);
    
    %Imsg = insertShape(Imsg, "FilledCircle", [loc, ...
    % repmat(10, length(loc), 1)],"Color","red","Opacity",1);

    %msg.Linear.Y = 0;
    %msg.Linear.Z = 0;
    %msg.Angular.X = 0;
    %msg.Angular.Y = 0;
    %msg.Angular.Z = 1;
    
    %send(pub, cmd_msg);
    
end
