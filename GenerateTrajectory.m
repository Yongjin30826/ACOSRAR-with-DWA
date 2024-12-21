function [x,traj]=GenerateTrajectory(x,vx,vy,vz,evaldt,model)
% �켣���ɺ���
% evaldt��ǰ��ģ��ʱ��; vt��ot��ǰ�ٶȺͽ��ٶ�;
global dt;
time=0;
% ����ֵ
u=[vx;vy;vz];
% ���˻��켣

numT = floor(evaldt/dt);
traj = zeros(length(x),numT);
for inum = 1:numT
    x=forward(x,u);
    traj(:,inum) = x;
end
