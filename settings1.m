% 设置动态障碍物的位置
numObs = 3; % 动态障碍物数量

% 设置障碍物
obs.V = [-6;6;0];  % [vx vy vz]
obs.T = 70;  % 往复运动的周期
obs.pos = [400;300;350]; % 取第一、二个路径点的中间位置
obs.R = 30; 
obsMove{1} = obs;  % 障碍物1设置完成

obs.V = [0;0;6];  % [vx vy vz]
obs.T = 50;  % 往复运动的周期
obs.pos = [ptsx(6)+ptsx(7);ptsy(6)+ptsy(7);ptsz(6)+ptsz(7)-20]/2; % 取第三、四个路径点的中间位置
obs.R = 20; 
obsMove{2} = obs;  % 障碍物2设置完成

% obs.V = [0;5;0];  % [vx vy vz]
% obs.T = 50;  % 往复运动的周期
% obs.pos = [ptsx(7)+ptsx(8);ptsy(7)+ptsy(8);ptsz(7)+ptsz(8)]/2; % 取第三、四个路径点的中间位置
% obs.R = 20; 
% obsMove{3} = obs;  % 障碍物3设置完成


% 圆柱体障碍物的参数
R1=130;  % Radius
x1 = 170; y1 = 330; z1 = 150; Hc1 = 350; % center

R2=130;  % Radius
x2 = 430; y2 = 150; z2 = 150;Hc2 = 350; % center

R3=50;  % Radius
x3 = 300; y3 = 240; z3 = 150;Hc3 = 200; % center

R4=70;  % Radius
x4 = 750; y4 = 640; z4 = 150;Hc4 = 180; % center

R5=70;  % Radius
x5 = 500; y5 = 450; z5 = 150;Hc5 = 350;% center

R6=100;  % Radius
x6 = 620; y6 = 730; z6 = 150;Hc6 = 350;% center

R7=100;  % Radius
x7 = 880; y7 = 550; z7 = 150;Hc7 = 350; % center

obsTmp.xyz = [x1;y1;z1];
obsTmp.R = R1;
obsTmp.Hc1 = Hc1;
obsStatic{1} = obsTmp;
obsTmp.xyz = [x2;y2;z2];
obsTmp.R = R2;
obsTmp.Hc2 = Hc2;
obsStatic{2} = obsTmp;
obsTmp.xyz = [x3;y3;z3];
obsTmp.R = R3;
obsTmp.Hc3 = Hc3;
obsStatic{3} = obsTmp;
obsTmp.xyz = [x4;y4;z4];
obsTmp.R = R4;
obsTmp.Hc4 = Hc4;
obsStatic{4} = obsTmp;
obsTmp.xyz = [x5;y5;z5];
obsTmp.R = R5;
obsTmp.Hc5 = Hc5;
obsStatic{5} = obsTmp;
obsTmp.xyz = [x6;y6;z6];
obsTmp.R = R6;
obsTmp.Hc6 = Hc6;
obsStatic{6} = obsTmp;