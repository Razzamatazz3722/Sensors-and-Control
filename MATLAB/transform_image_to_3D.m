function [X,Y,Z] = transform_image_to_3D(image_x, image_y, raw_camera_info, input_depth_image)

    %point of interest on image
    p_x = image_x;
    p_y = image_y;
    
    %raw depth image
    img = input_depth_image;
    
    %depth at point of interest
    depth = img(p_x, p_y);
    
    %camera calibration matrix
    K_matrix = [raw_camera_info.K(1,1), raw_camera_info.K(2,1), raw_camera_info.K(3,1);
                raw_camera_info.K(4,1), raw_camera_info.K(5,1), raw_camera_info.K(6,1);
                raw_camera_info.K(7,1), raw_camera_info.K(8,1), raw_camera_info.K(9,1);];
            
    K_inv = inv(K_matrix);
    
    image_matrix = [p_x*depth; p_y*depth; depth];
    
    X_cam = K_inv * image_matrix;

    X = X_cam(1,1);
    Y = X_cam(2,1);
    Z = X_cam(3,1);


end








