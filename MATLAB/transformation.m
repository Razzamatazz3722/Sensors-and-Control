clearvars
rosshutdown
rosinit

%while(1)

    sub = rossubscriber('/tb3_0/camera/depth/image_raw');
    pause(1);
    
    msg2 = receive(sub,10);
    
    %convert from ros image type to matlab image type
    img = readImage(msg2);

    p_x = 305;
    p_y = 231;
    
    depth = img(p_x,p_y);
    
    
    sub2 = rossubscriber('/tb3_0/camera/rgb/camera_info');
    pause(1);
    
    msg3 = receive(sub2,10);
    
    K_matrix = [msg3.K(1,1), msg3.K(2,1), msg3.K(3,1);
                msg3.K(4,1), msg3.K(5,1), msg3.K(6,1);
                msg3.K(7,1), msg3.K(8,1), msg3.K(9,1);];
    K_inv = inv(K_matrix);
    
    image_matrix = [p_x*depth; p_y*depth; depth];
    
    X_cam = K_inv * image_matrix;
    
    
    
    %ptcloud = rosmessage('sensor_msgs/PointCloud2');
    
    %ptcloud.PreserveStructureOnRead = true;
    
    %ptcloud = readXYZ(msg2);
    
    %disp(msg2.Data);
    
    %disp(ptcloud);
    
    %pcshow(ptcloud);
   
    
   
    
    
%end
