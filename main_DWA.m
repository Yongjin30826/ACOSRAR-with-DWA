% Revised: 12.2024
% This is ACOSRAR to optimize problems of UAV path planning in the 3D environment, based on the paper 
% "Ben Niu, Yongjin Wang, Jing Liu, and Gabriel Xiao-Guang Yue.Path Planning for Unmanned Aerial Vehicles in Complex 
%  Environment Based on an Improved Continuous Ant Colony Optimisation. Computers and Electrical Engineering"
clear
clc
close all
load dataACO.mat

pxyz = PlotSolution_new(BestPosition,model,smooth);

% 获取期望路径点
ptsx = pxyz(1,:);
ptsy = pxyz(2,:);
ptsz = pxyz(3,:);

figure(1)
hold on
route2 = pxyz'; % 获取粒子群的路径点

startPoint = route2(1,:); % 设置起点

% 无人机起点状态[x y z vx vy vz]
x=[startPoint(1),startPoint(2),startPoint(3),0 0 0]';%[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
result.x=zeros(6,100000); % 保留路径结果
settings1;
numPts = size(route2,1); % 获取动态路径点的数量

global dt; % 仿真时间
cutT = 0;
cnt = 0;
x_last = x;

% 画动态障碍物
[a,b,c]=sphere;%画圆（经纬度默认）
hObs = zeros(1,length(obsMove));
for iobs = 1:length(obsMove)
    obsTmp = obsMove{iobs};
    hObs(iobs) = mesh(obsTmp.R*a+obsTmp.pos(1),obsTmp.R*b+obsTmp.pos(2),obsTmp.R*c+obsTmp.pos(3));%画三维曲线图
end

hTraj = [];
for iPts = 2:numPts % 因为第一个点是起点，因此从第二个点开始
    goal = route2(iPts,:); % 293.6013  345.2300  200.0000
    dt=1;

    % 无人机模型参数,[最大X速度,最大Y速度,最大Z速度,最大X加速度,最大Y加速度,最大Z加速度,X速度分辨率,Y速度分辨率,Z速度分辨率]
    Kinematic = [6,6,6,3,3,3,0.5,0.5,2];
    % 动态窗口评价参数 [vx,vy,vz,xy航向权重,xz航向权重，障碍物距离权重,轨迹预测时间]
    evalParam=[0.0,0.0,0.0,0.01,0.01,0.002,0.01,3.0];

    for i=1:5000
        cutT = cutT + dt;
        cnt = cnt + 1;
        % DWA
        [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obsStatic,obsMove);

        % 机器人运动模型
        x=forward(x,u);
        x
        % 障碍物运动模型
        for iobs = 1:length(obsMove)
            obsMove{iobs}.pos = obsMove{iobs}.pos + obsMove{iobs}.V*dt*sin(2*pi*cutT/obsMove{iobs}.T);
            obsTmp = obsMove{iobs};
            set(hObs(iobs),'xData',obsTmp.R*a+obsTmp.pos(1),'yData',obsTmp.R*b+obsTmp.pos(2),'zData',obsTmp.R*c+obsTmp.pos(3));%画三维曲线图\
            axis([0,1000,0,1000,0,500]);
        end

        % 将路径点保存进result变量
        result.x(:,cnt) = x';
        % 是否到达目的地
        if iPts~=numPts
            if (norm(x(1:3)-goal')<10)
                disp('Arrive Goal!!');break;
            end
        else

            if (norm(x(1:3)-goal')<2)
                disp('Arrive Goal!!');break;
            end
        end

        % ====绘图部分===

        % 无人机轨迹
        plot3([x_last(1),x(1)],[x_last(2),x(2)],[x_last(3),x(3)],'--b','linewidth',2);hold on;

        % 探索轨迹
%         if ~isempty(traj)
%             if isempty(hTraj)
%                 for it=1:length(traj(:,1))/6
%                     ind=1+(it-1)*6;
%                     hTraj(it) = plot3(traj(ind,:),traj(ind+1,:),traj(ind+2,:),'-g');hold on;
%                 end
%             else
%                 for it=1:length(traj(:,1))/6
%                     ind=1+(it-1)*6;
%                     set(hTraj(it),'xData',traj(ind,:),'yData',traj(ind+1,:),'zData',traj(ind+2,:))
%                 end
%             end
%         end

        drawnow;
        x_last = x;
    end

end
















