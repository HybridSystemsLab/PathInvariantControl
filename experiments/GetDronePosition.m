function [result] = GetDronePosition(client,id)
% This function will get the data from Motive and extract the x, y, z
% coordinates from the incoming data.

R_x = [1 0 0; 0 cosd(-90) -sind(-90); 0 sind(-90) cosd(-90)];

R_z = [cosd(90) -sind(90) 0; sind(90) cosd(90) 0; 0 0 1];
frameData = client.GetLastFrameOfData();

%Get the time stamp
time_stamp = frameData.fTimestamp;

%Get the marker data
drone_pos = frameData.RigidBodies(id);

%ball_pos = frameData.OtherMarkers(id);
if length(drone_pos) > 0
    
    %% Motive coordiate frame
    %        ^ z
    %        |
    %        |
    % x <----O y(pointing up)
    %
    
    %% Our coordiate frame
    % 
    % x <----O z(pointing up)
    %        |
    %        |
    %        v y
    
    %p = [drone_pos.x; drone_pos.y; drone_pos.z];
    
    %p_new = R_x * (R_z * p);
%     x_d = -p(3);
%     y_d = -p(1); 
%     z_d = p(2);

    x_d = drone_pos.x;
    y_d = -drone_pos.z; 
    z_d = drone_pos.y;
    q = [drone_pos.qx, drone_pos.qy, drone_pos.qz, drone_pos.qw];
    Eul_ang = quat2eul(q);
    yaw = -Eul_ang(2);
    if Eul_ang(1) <= 0
        pitch = pi + Eul_ang(1);
    else
        pitch = Eul_ang(1)-pi;
    end
%     pitch = Eul_ang(1);
    roll = Eul_ang(3);
else
    x_d = 0;
    y_d = 0;
    z_d = 0;
    yaw = 0;
end

result = [time_stamp, x_d, y_d, z_d, pitch, roll, yaw];

end


