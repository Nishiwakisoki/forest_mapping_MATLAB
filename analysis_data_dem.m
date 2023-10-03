%% analysis the data of DEM
DEM = pcread("forest_result/akaezomatu_map_503s_DEM.pcd");
show_figure_pointcloud(DEM)

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