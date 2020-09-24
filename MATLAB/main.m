rosshutdown
rosinit

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

    [msg,~,loc] = readBarcode(new_I,"QR-CODE");
    Imsg = insertShape(new_I, "FilledCircle", [loc, repmat(10, length(loc), 1)],"Color","red","Opacity",1);
    imshow(Imsg) 
    
    %[msg,~,loc] = readBarcode(img, "QR-CODE");
    %Imsg = insertShape(img, "FilledCircle", [loc, repmat(10, length(loc), 1)], "Color", "red", "Opacity", 1);
      
    %imshow(Imsg)
    
    %setup ros publisher
    %pub = rospublisher('/follower/cmd_vel', 'geometry_msgs/Twist');
    
    %msg = rosmessage(pub);
    %msg.Linear.X = 0;
    %msg.Linear.Y = 0;
    %msg.Linear.Z = 0;
    %msg.Angular.X = 0;
    %msg.Angular.Y = 0;
    %msg.Angular.Z = 1;
    
    %send(pub, msg);
    
end



    
   % xyText =  loc(1,:);
    %Imsg = insertText(img,xyText,msg,"BoxOpacity",1,"FontSize",25);
    
    %Imsg = insertShape(Imsg, "FilledCircle", [loc, ...
    % repmat(10, length(loc), 1)],"Color","red","Opacity",1);
