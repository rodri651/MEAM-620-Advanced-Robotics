function [vel,omg] = estimate_vel(sensor, varargin)

alpha = 0.2;
alpha_vel = 0.3;
alpha_omg = 0.7;
K=[311.0520 0        201.8724;
 0        311.3885 113.6210;
 0        0        1];

persistent pointTracker s_t_old image_old V_old delta_t_old
if isempty(s_t_old) && sensor.is_ready==1 && ~isempty(sensor.id)
    corners = detectFASTFeatures(sensor.img);
    pointTracker = vision.PointTracker; 
    initialize(pointTracker,corners.Location,sensor.img);
    omg = zeros(3,1);
    vel = zeros(3,1);
    
    s_t_old = sensor.t; 
    image_old = sensor.img;
    delta_t_old = s_t_old;
elseif sensor.is_ready==1 && ~isempty(sensor.id)
   
    corners_unstrong = detectFASTFeatures(image_old);
    corners = corners_unstrong.selectStrongest(100);
    pointTracker = vision.PointTracker; 
    initialize(pointTracker,corners.Location,image_old);
    old_camera_points = K\[corners.Location,ones(corners.Count,1)]';
    new_image_points= pointTracker(sensor.img);
    [H,R,t]=compute_homography(sensor);
    new_image_points = [new_image_points,ones( size(new_image_points,1),1 )]';
    
    new_world_points = H\new_image_points;
    new_world_points = new_world_points./new_world_points(3,:);
    new_world_points(3,:) = zeros(1,size(new_world_points,2));
    
    z = R(3,:)*new_world_points + t(3);
    
    delta_t = sensor.t-s_t_old;
    s_t = alpha*delta_t + (1-alpha)*delta_t_old;
    s_t_old = sensor.t; 
    delta_t_old = s_t;
    new_camera_points = K\new_image_points;
    
    displacement = new_camera_points - old_camera_points;
    vel = displacement(1:2,:)/s_t;
    [V] = run_ransac(z,new_camera_points,vel);
    image_old = sensor.img;
    
    omg = R\V(4:6);
    vel = R\(V(1:3));
    if isempty(V_old)
        V_old = [vel;omg];
    else 
        if abs(vel(1)-V_old(1))<0.5
        vel(1) = alpha_vel*vel(1) + (1-alpha_vel)*V_old(1);
        else 
            vel(1) = V_old(1);
        end
        if abs(vel(2)-V_old(2))<0.5
        vel(2) = alpha_vel*vel(2) + (1-alpha_vel)*V_old(2);
        else 
            vel(2) = V_old(2);
        end
        if abs(vel(3)-V_old(3))<0.5
        vel(3) = alpha_vel*vel(3) + (1-alpha_vel)*V_old(3);
        else 
            vel(3) = V_old(3);
        end
        if abs(omg(1)-V_old(4))<1.1
        omg(1) = alpha_omg*omg(1) + (1-alpha_omg)*V_old(4);
       else 
            omg(1) = V_old(4);
        end
        if abs(omg(2)-V_old(5))<0.8
        omg(2) = alpha_omg*omg(2) + (1-alpha_omg)*V_old(5);
        else 
            omg(2) = V_old(5);
        end
        if abs(omg(3)-V_old(6))<0.8

        else 
            omg(3) = V_old(6);
        end
        V_old = [vel;omg];
        
    end

 
    
else
    if ~isempty(V_old)
    vel = V_old(1:3);
    omg = V_old(4:6);
    image_old = sensor.img;
    
    else 
    image_old = sensor.img;
    vel = zeros(3,1);
    omg = zeros(3,1);
    
    end
    
end



end
