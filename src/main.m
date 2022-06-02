addpath('utilities')
more off
close all
clear all

% 显示结果
window = true;

% 加载激光信息和机器人位姿
load("../data/laser")
graphics_toolkit("qt");
% 提取机器人位姿: Nx3 矩阵, 行向量为[x y theta]
poses = [laser.pose];
poses = reshape(poses, 3, size(poses, 2) / 3);

% 初始占据栅格概率
prior = 0.5;

% 激光模型相关概率
probOcc = 0.9;
probFree = 0.35;

% 栅格尺寸(米)
gridSize = 0.25;

% 设定地图边界并初始化地图
border = 30;
robXMin = min(poses(:,1));
robXMax = max(poses(:,1));
robYMin = min(poses(:,2));
robYMax = max(poses(:,2));
mapBox = [robXMin-border robXMax+border robYMin-border robYMax+border];
offsetX = mapBox(1);
offsetY = mapBox(3);
mapSizeMeters = [mapBox(2)-offsetX mapBox(4)-offsetY];
mapSize = ceil([mapSizeMeters/gridSize]);

% 计算先验概率
logOddsPrior = prob_to_log_odds(prior);

% 初始化占据栅格的logOdds值
map = logOddsPrior*ones(mapSize);
disp('Map initialized. Map size:'), disp(size(map))

% 当从世界转换到地图坐标系时的地图补偿值
offset = [offsetX; offsetY];

% main loop
for t=1:size(poses,2)
    t
    % 机器人在世界坐标系下的位姿 3xN
    robPose = [poses(1,t); poses(2,t); poses(3,t)];

    % t 时刻的激光扫描
    sc = laser(1,t);

    % 计算地图更新，包含了用于地图更新的logOdds值
    [mapUpdate, robPoseMapFrame, laserEndPntsMapFrame] = inv_sensor_model(map, sc, robPose, gridSize, offset, probOcc, probFree, t);

    mapUpdate = mapUpdate - logOddsPrior*ones(size(map));

    % 更新地图
    map = map + mapUpdate;

    % 画出当前地图和机器人轨迹
    plot_map(map, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t, window);

end
