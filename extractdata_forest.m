clc
clear
% startの瞬間は110秒から
bag = rosbag('/mnt/sda1/mikasa_uav/2023-08-18-19-11-37.bag');
StartTime = bag.StartTime;
%
attitude = select(bag,'Topic','/dji_osdk_ros/attitude');
attitude_data = readMessages(attitude);
gps = select(bag,'Topic','/dji_osdk_ros/rtk_position');
gps_data = readMessages(gps);
ouster1 = select(bag,'Topic','/ouster/points');
ouster_data1 = readMessages(ouster1);
% gpsstart = cellfun(@(m) m.Header.Stamp.Sec + m.Header.Stamp.Nsec/1000000000.0,gps_data(1));
ousstart = cellfun(@(m) m.Header.Stamp.Sec + m.Header.Stamp.Nsec/1000000000.0,ouster_data1(1));
gpsTime = cellfun(@(m) m.Header.Stamp.Sec + m.Header.Stamp.Nsec/1000000000.0-StartTime,gps_data);
% ousTime = cellfun(@(m) m.Header.Stamp.Sec + m.Header.Stamp.Nsec/1000000000.0-ousstart-1.655104024911193e+09+StartTime,ouster_data1);
attitudeTime = cellfun(@(m) m.Header.Stamp.Sec + m.Header.Stamp.Nsec/1000000000.0-StartTime,attitude_data);
ousTime = cellfun(@(m) m.Header.Stamp.Sec + m.Header.Stamp.Nsec/1000000000.0-StartTime,ouster_data1);
%%
% rng('default');
% % player
% xlimits = [-100 100];
% ylimits = [-100 100];
% zlimits = [-100 100];
% 
% % Create a pcplayer object to visualize the lidar scans
% lidarPlayer = pcplayer(xlimits,ylimits,zlimits);
% 
% % Customize the pcplayer axis labels
% xlabel(lidarPlayer.Axes,'X (m)')
% ylabel(lidarPlayer.Axes,'Y (m)')
% zlabel(lidarPlayer.Axes,'Z (m)')
% title(lidarPlayer.Axes,'Lidar Scans')
% for i = 2000 : 9500
%     viewcloud = pointCloud(readXYZ(ouster_data1{i,1}));
%     view(lidarPlayer,viewcloud);
%     % pause(0.1)
% end
%
l = 1;
k = 1;
for i = 1:size(gpsTime,1)
    [val1,indx1] = min(abs(gpsTime(i) - ousTime));
    [val2,indx2] = min(abs(gpsTime(i) - attitudeTime));
    if val1 <= 0.1
        ousteridx(k,:) = [i indx1];
        k = k + 1;
    end
     if val2 <= 0.1
        imuidx(l,:) = [i indx2];
        l = l + 1;
    end
end

%%
pcFilesPath = fullfile('/mnt/sda1/mikasa_50m5s');
if ~exist(pcFilesPath,'dir')
    mkdir(pcFilesPath);
end


for i = 1:length(ousteridx)
    pc = pointCloud(readXYZ(ouster_data1{ousteridx(i,2)}));
    n_strPadded = sprintf('%04d',i) ;
    pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
    pcwrite(pc,pcFileName);
end
%
% k = 1;
% l = 1;
% gpsidx = [];
% imuidx = [];
% for i = 1:size(ousTime,1)
%     [val1,indx1] = min(abs(ousTime(i) - gpsTime));
%     [val2,indx2] = min(abs(ousTime(i) - attitudeTime));
%     if val1 <= 0.1
%         gpsidx(k,:) = [i indx1];
%         k = k + 1;
%     end
%      if val2 <= 0.1
%         imuidx(l,:) = [i indx2];
%         l = l + 1;
%     end
%% end
temp =[];
merge_data = [];
for i = 1:length(imuidx)
    temp = [double(attitude_data{imuidx(i,2)}.Quaternion.X),double(attitude_data{imuidx(i,2)}.Quaternion.Y),double(attitude_data{imuidx(i,2)}.Quaternion.Z),double(attitude_data{imuidx(i,2)}.Quaternion.W), double(gps_data{i}.Latitude),double(gps_data{i}.Longitude),gps_data{i}.Altitude];
    merge_data = cat(1,merge_data,temp);
end
GpsAltitudeMetrics = table('Size', [length(merge_data(:,1)),7], ...
            'VariableTypes', {'double', 'double', 'double', 'double', 'double','double','double'}, ...
            'VariableNames',{'X', 'Y', 'Z', 'W', 'latitude','longitude','Altitude'});
GpsAltitudeMetrics.X = merge_data(:,1);
GpsAltitudeMetrics.Y = merge_data(:,2);
GpsAltitudeMetrics.Z = merge_data(:,3);
GpsAltitudeMetrics.W = merge_data(:,4);
GpsAltitudeMetrics.latitude = merge_data(:,5);
GpsAltitudeMetrics.longitude = merge_data(:,6);
GpsAltitudeMetrics.Altitude = merge_data(:,7);

writetable(GpsAltitudeMetrics,"csv/mikasa_50m5ms.csv");

GPSAltData = readmatrix("csv/mikasa_50m5ms.csv");
%creat UTM
zone = '54T';
[ellipsoid,estr] = utmgeoid(zone);
utmstruct = defaultm('utm');
utmstruct.zone =zone;
utmstruct.geoid = ellipsoid;
utmstruct = defaultm(utmstruct);
mergeUTM_data = [];
for i=1:length(GPSAltData)
    [east0,north0,h0]=mfwdtran(utmstruct,GPSAltData(i,5),GPSAltData(i,6),GPSAltData(i,7));
    tempUTM = [east0,north0,h0];
    mergeUTM_data = cat(1,mergeUTM_data,tempUTM);
end
mergemap_data = [mergeUTM_data(:,1),mergeUTM_data(:,2),mergeUTM_data(:,3),GPSAltData(:,1),GPSAltData(:,2),GPSAltData(:,3),GPSAltData(:,4)];
odomMetrics = table('Size', [length(mergemap_data(:,1)),7], ...
            'VariableTypes', {'double', 'double', 'double', 'double', 'double','double','double'}, ...
            'VariableNames',{'X', 'Y', 'Z', 'W', 'mapx','mapy','Altitude'});
odomMetrics.X = mergemap_data(:,4);
odomMetrics.Y = mergemap_data(:,5);
odomMetrics.Z = mergemap_data(:,6);
odomMetrics.W = mergemap_data(:,7);
odomMetrics.mapx = mergemap_data(:,1);
odomMetrics.mapy = mergemap_data(:,2);
odomMetrics.Altitude = mergemap_data(:,3);
writetable(odomMetrics,"csv/odom_mikasa_50m5ms.csv")

% scatter(mergemap_data(:,1),mergemap_data(:,2))

%%
rng('default');
% player
xlimits = [-100 100];
ylimits = [-100 100];
zlimits = [-100 100];

% Create a pcplayer object to visualize the lidar scans
lidarPlayer = pcplayer(xlimits,ylimits,zlimits);

% Customize the pcplayer axis labels
xlabel(lidarPlayer.Axes,'X (m)')
ylabel(lidarPlayer.Axes,'Y (m)')
zlabel(lidarPlayer.Axes,'Z (m)')
title(lidarPlayer.Axes,'Lidar Scans')
for i = 1000 : 2500
    viewcloud = pointCloud(readXYZ(ouster_data1{i}));
    view(lidarPlayer,viewcloud);
    % pause(0.1)
end