rosshutdown
rosinit

while(1)
    sub = rossubscriber('/follower/camera/rgb/image_raw');
    pause(1);

    msg2 = receive(sub,10);
    %convert from ros image type to matlab image type
    img = readImage(msg2);
    roi = [470,300,720,620];
    [msg,~,loc] = readBarcode(img,roi,"QR-CODE");
    
    xyText =  loc(2,:);
    Imsg = insertText(img,xyText,msg,"BoxOpacity",1,"FontSize",25);
    
    Imsg = insertShape(Imsg, "FilledCircle", [loc, ...
     repmat(10, length(loc), 1)],"Color","red","Opacity",1);
    
    imshow(Imsg)
    
    
    
    
    
    %pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
    
    %msg = rosmessage(pub);
    %msg.Linear.X = 0;
    %msg.Linear.Y = 0;
    %msg.Linear.Z = 0;
    %msg.Angular.X = 0;
    %msg.Angular.Y = 0;
    %msg.Angular.Z = 1;
    
    %send(pub, msg);
    
end