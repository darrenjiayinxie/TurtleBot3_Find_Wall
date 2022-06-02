function [mapUpdate, robPoseMapFrame, laserEndPntsMapFrame] = inv_sensor_model(map, scan, robPose, gridSize, offset, probOcc, probFree, t)
% 基于激光雷达的逆传感器模型求解log odds值, 并用于更新map

%%%%%% 输入值 %%%%%%
% (1) map: 包含了地图中每个栅格的占据值(IN LOG ODDS)的矩阵
% (2) scan: 在当前时刻激光的扫描值， 包含每一束激光的范围值读数
% (3) robPose: 机器人在世界坐标系下的位姿, 3xN 矩阵
% (4) gridSize: 地图栅格值的大小（米）
% (5) offset = [offsetX; offsetY]:  当空间中的点转化为地图坐标系时需要减去的偏移量
% (6) probOcc: 当一束激光击中栅格时，该栅格被障碍物占据的概率
% (7) probFree: 当一束激光击中栅格时，该栅格是自由的（不被障碍物占据）的概率

%%%%%% 输出值 %%%%%%
% (1) mapUpdate: 更新过的map矩阵, 受当前激光扫描值影响。不受影响的栅格值必须为零
% (2) robPoseMapFrame:  机器人在地图坐标系下的位姿
% (3) laserEndPntsMapFrame: 每一束激光末端的地图坐标值(也被用于可视化).

% 初始化 mapUpdate.
mapUpdate = zeros(size(map));

% 将Robot pose 转换为齐次变换矩阵.
robTrans = v2t(robPose);

% 使用 world_to_map_coordinates 来计算 robPoseMapFrame.
robPoseMapFrame = world_to_map_coordinates(robPose, gridSize, offset);

% 计算激光束末端的笛卡尔坐标值
% 设定第三个参数为 'true' 来使用一半的激光束从而加速（debug）
laserEndPnts = robotlaser_as_cartesian(scan, 30, false);

% 计算激光束末端在世界坐标系的坐标值
laserEndPnts = robTrans*laserEndPnts;
% 基于 laserEndPnts 和 world_to_map_coordinates 计算 laserEndPntsMapFrame.
laserEndPntsMapFrame = world_to_map_coordinates(laserEndPnts, gridSize, offset);
% 使用unique来去重
laserEndPntsMapFrame = unique(laserEndPntsMapFrame', 'rows')';

% 初始化 freeCells 矩阵， 即激光束可自由穿过的栅格的坐标值
freeCells = [];

% 遍历激光束来计算freeCells.
% 使用 bresenham 方法来计算激光束在地图坐标系上经过的点的坐标
for sc=1:columns(laserEndPntsMapFrame)

  % 计算 freecells沿着激光束到末端所经过点的所有地图坐标
  [X,Y] = bresenham([robPoseMapFrame(1), robPoseMapFrame(2); laserEndPntsMapFrame(1,sc), laserEndPntsMapFrame(2,sc)]);

  % 把它们加入freeCells
  freeCells = [freeCells; [X(1:end-1)', Y(1:end-1)']];

endfor

% 保证矩阵在boundary中
freeCells = abs(freeCells);
freeCells = max(1, freeCells);
laserEndPntsMapFrame = abs(laserEndPntsMapFrame);
laserEndPntsMapFrame = max(1, laserEndPntsMapFrame);

freeCells(:,1) = min(size(mapUpdate,1), freeCells(:,1));
freeCells(:,2) = min(size(mapUpdate,2), freeCells(:,2));

laserEndPntsMapFrame(1,:) = min(size(mapUpdate,1), laserEndPntsMapFrame(1,:));
laserEndPntsMapFrame(2,:) = min(size(mapUpdate,2), laserEndPntsMapFrame(2,:));


% 根据probFree更新freeCells中的log odds值
idx = sub2ind(size(mapUpdate), freeCells(:,1), freeCells(:,2));
mapUpdate(idx) = prob_to_log_odds(probFree);

% 根据probOcc更新laserEndPntsMapFrame中的log odds值

idx = sub2ind(size(mapUpdate), laserEndPntsMapFrame(1,:), laserEndPntsMapFrame(2,:));
mapUpdate(idx) = prob_to_log_odds(probOcc);

end
