% Revised: 12.2024
% This is ACOSRAR to optimize problems of UAV path planning in the 3D environment, based on the paper 
% "Ben Niu, Yongjin Wang, Jing Liu, and Gabriel Xiao-Guang Yue.Path Planning for Unmanned Aerial Vehicles in Complex 
%  Environment Based on an Improved Continuous Ant Colony Optimisation. Computers and Electrical Engineering"
clear
clc
close all
load dataACO.mat

pxyz = PlotSolution_new(BestPosition,model,smooth);

% ��ȡ����·����
ptsx = pxyz(1,:);
ptsy = pxyz(2,:);
ptsz = pxyz(3,:);

figure(1)
hold on
route2 = pxyz'; % ��ȡ����Ⱥ��·����

startPoint = route2(1,:); % �������

% ���˻����״̬[x y z vx vy vz]
x=[startPoint(1),startPoint(2),startPoint(3),0 0 0]';%[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
result.x=zeros(6,100000); % ����·�����
settings1;
numPts = size(route2,1); % ��ȡ��̬·���������

global dt; % ����ʱ��
cutT = 0;
cnt = 0;
x_last = x;

% ����̬�ϰ���
[a,b,c]=sphere;%��Բ����γ��Ĭ�ϣ�
hObs = zeros(1,length(obsMove));
for iobs = 1:length(obsMove)
    obsTmp = obsMove{iobs};
    hObs(iobs) = mesh(obsTmp.R*a+obsTmp.pos(1),obsTmp.R*b+obsTmp.pos(2),obsTmp.R*c+obsTmp.pos(3));%����ά����ͼ
end

hTraj = [];
for iPts = 2:numPts % ��Ϊ��һ��������㣬��˴ӵڶ����㿪ʼ
    goal = route2(iPts,:); % 293.6013  345.2300  200.0000
    dt=1;

    % ���˻�ģ�Ͳ���,[���X�ٶ�,���Y�ٶ�,���Z�ٶ�,���X���ٶ�,���Y���ٶ�,���Z���ٶ�,X�ٶȷֱ���,Y�ٶȷֱ���,Z�ٶȷֱ���]
    Kinematic = [6,6,6,3,3,3,0.5,0.5,2];
    % ��̬�������۲��� [vx,vy,vz,xy����Ȩ��,xz����Ȩ�أ��ϰ������Ȩ��,�켣Ԥ��ʱ��]
    evalParam=[0.0,0.0,0.0,0.01,0.01,0.002,0.01,3.0];

    for i=1:5000
        cutT = cutT + dt;
        cnt = cnt + 1;
        % DWA
        [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obsStatic,obsMove);

        % �������˶�ģ��
        x=forward(x,u);
        x
        % �ϰ����˶�ģ��
        for iobs = 1:length(obsMove)
            obsMove{iobs}.pos = obsMove{iobs}.pos + obsMove{iobs}.V*dt*sin(2*pi*cutT/obsMove{iobs}.T);
            obsTmp = obsMove{iobs};
            set(hObs(iobs),'xData',obsTmp.R*a+obsTmp.pos(1),'yData',obsTmp.R*b+obsTmp.pos(2),'zData',obsTmp.R*c+obsTmp.pos(3));%����ά����ͼ\
            axis([0,1000,0,1000,0,500]);
        end

        % ��·���㱣���result����
        result.x(:,cnt) = x';
        % �Ƿ񵽴�Ŀ�ĵ�
        if iPts~=numPts
            if (norm(x(1:3)-goal')<10)
                disp('Arrive Goal!!');break;
            end
        else

            if (norm(x(1:3)-goal')<2)
                disp('Arrive Goal!!');break;
            end
        end

        % ====��ͼ����===

        % ���˻��켣
        plot3([x_last(1),x(1)],[x_last(2),x(2)],[x_last(3),x(3)],'--b','linewidth',2);hold on;

        % ̽���켣
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
















