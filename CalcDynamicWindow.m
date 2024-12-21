function Vr=CalcDynamicWindow(x,Kinematic)

global dt;
% 无人机x方向、y方向、z方向速度的最大最小范围
Vs=[-Kinematic(1) Kinematic(1) -Kinematic(2) Kinematic(2) -Kinematic(3) Kinematic(3)];
% 根据当前速度以及加速度限制计算的动态窗口
Vd=[x(4)-Kinematic(4)*dt x(4)+Kinematic(4)*dt x(5)-Kinematic(5)*dt x(5)+Kinematic(5)*dt x(6)-Kinematic(6)*dt x(6)+Kinematic(6)*dt];
% 最终的Dynamic Window
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4)) max(Vtmp(:,5)) min(Vtmp(:,6))];