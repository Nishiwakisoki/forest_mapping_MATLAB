pc = pcread("/home/robotics/MATLAB/usr/forest_result/todo_map_503s_trim.pcd");
pcshow(pc)
[groundPtCloud, nonGroundPtCloud] = pc_ground_extraction(pc);
show_figure_pointcloud(groundPtCloud)
show_figure_pointcloud_pair(groundPtCloud,nonGroundPtCloud)

%%
pcwrite(groundPtCloud,"forest_result/todo_map_503s_DEM.pcd");
pcwrite(nonGroundPtCloud,"forest_result/todo_map_503s_nonground.pcd")
function [groundPtCloud,nongroundPtCloud] = pc_ground_extraction(pc)
% -- read pointcloud and normalized --
% Segment Ground and extract non-ground and ground points
groundPtsIdx = segmentGroundSMRF(pc,2,"SlopeThreshold",0.1,"MaxWindowRadius",20,"ElevationThreshold",1.0);
groundPtCloud = select(pc,groundPtsIdx);
nongroundPtCloud = select(pc,~groundPtsIdx);
end

function show_figure_pointcloud(pc)
hFigTraj = figure;
axTraj = axes(Parent=hFigTraj,Color='black');
pcshow(pc,Parent=axTraj)
hold on
% pcshow(estimatedTraj,'red',MarkerSize=150,Parent=axTraj)
% trans_data = [translations(start:endframe,1),translations(start:endframe,2),translations(start:endframe,3)];
% scatter3(estimatedTraj(:,1),estimatedTraj(:,2),estimatedTraj(:,3),'green',Parent=axTraj)
% scatter3(trans_data(:,1),trans_data(:,2),trans_data(:,3),'red',Parent=axTraj)
% Customize axis labels
axis on
xlabel(axTraj,'X (m)')
ylabel(axTraj,'Y (m)')
zlabel(axTraj,'Z (m)')
grid on
hold off
end

function show_figure_pointcloud_pair(groundPtCloud,nonGroundPtCloud)
hFigTraj = figure;
axTraj = axes(Parent=hFigTraj,Color='black');
pcshowpair(groundPtCloud,nonGroundPtCloud,Parent=axTraj)
hold on
axis on
xlabel(axTraj,'X (m)')
ylabel(axTraj,'Y (m)')
zlabel(axTraj,'Z (m)')
grid on
hold off
end