function [x,traj]=GenerateTrajectory(x,vx,vy,vz,evaldt,model)
% 轨迹生成函数
% evaldt：前向模拟时间; vt、ot当前速度和角速度;
global dt;
time=0;
% 输入值
u=[vx;vy;vz];
% 无人机轨迹

numT = floor(evaldt/dt);
traj = zeros(length(x),numT);
for inum = 1:numT
    x=forward(x,u);
    traj(:,inum) = x;
end
