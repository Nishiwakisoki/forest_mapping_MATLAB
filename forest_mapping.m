clc
clear
pcd_list = dir("/mnt/sda1/mikasa_50m3s/*.pcd");
odom_azumai = readmatrix('/home/robotics/MATLAB/usr/csv/odom_mikasa_50m3ms.csv');
% 必要に応じて初期化
pccell = cell(2500, 1);
minLidarRange = 1;

parfor n = 1:2500
    filename = ['/mnt/sda1/mikasa_50m3s/' pcd_list(n).name];
    scanPtcloud = pcread(filename);
    scanPtIdx = scanPtcloud.Location;
    ind = (-minLidarRange < scanPtIdx(:,1) & scanPtIdx(:,1) < minLidarRange ...
        & -minLidarRange  < scanPtIdx(:,2) & scanPtIdx(:,2) < minLidarRange ...
        & -minLidarRange  < scanPtIdx(:,3) & scanPtIdx(:,3) < minLidarRange);
    
    fixed = pointCloud(scanPtIdx(~ind,:));
    pccell{n} = fixed;
end
%%
rng('default');
% player
xlimits = [-100 100];
ylimits = [-100 100];
zlimits = [-60 10];

% Create a pcplayer object to visualize the lidar scans
lidarPlayer = pcplayer(xlimits,ylimits,zlimits);

% Customize the pcplayer axis labels
xlabel(lidarPlayer.Axes,'X (m)')
ylabel(lidarPlayer.Axes,'Y (m)')
zlabel(lidarPlayer.Axes,'Z (m)')
title(lidarPlayer.Axes,'Lidar Scans')
for i = 1030 : 1220
    viewcloud = pccell{i};
    view(lidarPlayer,viewcloud);
    pause(0.1)
end
%
%%
% skipframe = 2;
% start = 1500;
% endframe = 3000;
% 
% odomstart = 1;
% odomend = 10000;
skipframe = 1;

%akaezo
% start = 560;
% endframe = 730;

% karamatu
start = 1020;
endframe = 1220;

% todo
% start = 1500;
% endframe = 1700;

odomstart = 1;
odomend = 1900;
q = [odom_azumai(odomstart:odomend,4),odom_azumai(odomstart:odomend,1),odom_azumai(odomstart:odomend,2),odom_azumai(odomstart:odomend,3)];
eastX = odom_azumai(odomstart:odomend,5)- odom_azumai(odomstart,5);
northY = odom_azumai(odomstart:odomend,6)- odom_azumai(odomstart,6);
upZ = odom_azumai(odomstart:odomend,7)- odom_azumai(odomstart,7);
% scatter3(eastX,northY,upZ)
eulXYZ  = quat2eul(q, 'XYZ');
translations = [eastX,northY,upZ];
ptCloudScene = pointCloud.empty(0);
% t = -pi/2;
% Ry = [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
tx = [0 0 0];
Rx = [0 90 0];
estimatedTraj = zeros(endframe - start + 1, 3); % 絶対的な経路を格納するための行列
% pcprocesses = [];
for n = start:skipframe:endframe
    ptcloud = pccell{n};
    tformmoving = rigidtform3d(Rx,tx);
    ptclouda = pctransform(ptcloud,tformmoving);
    ptclouda = pcdenoise(ptclouda);
    translation = [translations(n,1),translations(n,2),translations(n,3)] ;
    % translation = [0,0,0];
    R = [rad2deg(eulXYZ(n,1)) rad2deg(eulXYZ(n,2)) rad2deg(eulXYZ(n,3))];
    tform = rigidtform3d(R,translation);
    ptCloudtrans= pctransform(ptclouda,tform);
    % view(lidarPlayer,ptCloudtrans);
    if n == start
        moving = ptCloudtrans;
        % ptCloudScene = moving;
        scanAccepted = 1;
        estimatedTraj(n - start + 1, :) = translation; % 初期位置を格納
    elseif n == (start + skipframe)
        fixed = moving;
        moving = ptCloudtrans;
        % fixedDown = pcdownsample(fixed,"random",0.1);
        % moving = ptCloudtrans;
        % movingDown = pcdownsample(moving,"random",0.1);
        rotationR = eul2rotm(deg2rad(R));
        rotationR_prev = eul2rotm(deg2rad(R_prev));
        % 2つの回転行列の相対関係を計算
        relative_rotation_matrix = rotationR * rotationR_prev';
        % 相対関係の回転行列をオイラー角に変換
        relative_euler_angles = rotm2eul(relative_rotation_matrix);

        tformIninal = rigidtform3d(relative_euler_angles,translation - translation_prev);
        tform = pcregistericp(moving,fixed,Metric="planeToPlane",InitialTransform=tformIninal);
        % tform.Translation = [0,0,0];
        % tform = pcregisterndt(moving,fixed,3.0);
        ptCloudAligned = pctransform(moving,tform);
        mergeSize = 0.01;
        ptCloudScene = pcmerge(moving,ptCloudAligned,mergeSize);
        accumTform = tform;
        ptCloudScene = pcmerge(ptCloudScene,ptCloudAligned,mergeSize);
        estimatedTraj(n - start + 1, :) = tform.Translation;
    else
        fixed = moving;
        moving = ptCloudtrans;
        % % fixedDown = pcdownsample(fixed,"random",0.1);
        % movingDown = pcdownsample(moving,"random",0.1);
        rotationR = eul2rotm(deg2rad(R));
        rotationR_prev = eul2rotm(deg2rad(R_prev));
        % 2つの回転行列の相対関係を計算
        relative_rotation_matrix = rotationR * rotationR_prev';
        % 相対関係の回転行列をオイラー角に変換
        relative_euler_angles = rotm2eul(relative_rotation_matrix);
        tformIninal = rigidtform3d(R - R_prev,translation - translation_prev);
        tform = pcregistericp(moving,fixed,Metric="planeToPlane",InitialTransform=tformIninal);
        % tform.Translation = [0,0,0];
        % tform = rigidtform3d(tform.R,tform.Translation);
        accumTform = rigidtform3d(accumTform.A * tform.A);
        ptCloudAligned = pctransform(moving,accumTform);
        mergeSize = 0.01;
        % Update the world scene.
        estimatedTraj(n - start + 1, :) = tform.Translation+translation;
        ptCloudScene = pcmerge(ptCloudScene,ptCloudAligned,mergeSize);
    end
    R_prev = R;
    translation_prev = translation;
    scatter(estimatedTraj(:,1),estimatedTraj(:,2))
    drawnow
end
%%
hFigTraj = figure;
axTraj = axes(Parent=hFigTraj,Color='black');
pcshow(ptCloudScene,Parent=axTraj)
hold on
% pcshow(estimatedTraj,'red',MarkerSize=150,Parent=axTraj)
trans_data = [translations(start:endframe,1),translations(start:endframe,2),translations(start:endframe,3)];
scatter3(estimatedTraj(:,1),estimatedTraj(:,2),estimatedTraj(:,3),'green',Parent=axTraj)
scatter3(trans_data(:,1),trans_data(:,2),trans_data(:,3),'red',Parent=axTraj)
% Customize axis labels
axis on
xlabel(axTraj,'X (m)')
ylabel(axTraj,'Y (m)')
zlabel(axTraj,'Z (m)')
grid on
hold off
%%
scatter3(estimatedTraj(:,1),estimatedTraj(:,2),estimatedTraj(:,3),'green')
hold on
scatter3(trans_data(:,1),trans_data(:,2),trans_data(:,3),'red')
% Customize axis labels
axis on
% xlabel(axTraj,'X (m)')
% ylabel(axTraj,'Y (m)')
% zlabel(axTraj,'Z (m)')
grid on
hold off
pcwrite(ptCloudScene,"forest_result/karamatu_map_503s.pcd");