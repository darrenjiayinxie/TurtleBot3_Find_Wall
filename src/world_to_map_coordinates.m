function [pntsMap] = world_to_map_coordinates(pntsWorld, gridSize, offset)
%%%%%% 输入值 %%%%%%
% 将点从世界坐标系转换到地图坐标系
% pntsWorld： 是一个矩阵包含了N个点，列向量代表了点的世界坐标(米)
% gridSize： 栅格尺寸(米)
% offset = [offsetX; offsetY]： 当空间中的点转化为地图坐标系时需要减去的偏移量 

%%%%%% 输出值 %%%%%%
% pntsMap： 2xN 矩阵 对应的点在地图坐标系中的坐标

% 计算 pntsMap
pntsMap = pntsWorld;
pntsMap(1:2, :) = ceil((pntsWorld(1:2, :) - offset) / gridSize);

end