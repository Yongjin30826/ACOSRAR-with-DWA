function Vr=CalcDynamicWindow(x,Kinematic)

global dt;
% ���˻�x����y����z�����ٶȵ������С��Χ
Vs=[-Kinematic(1) Kinematic(1) -Kinematic(2) Kinematic(2) -Kinematic(3) Kinematic(3)];
% ���ݵ�ǰ�ٶ��Լ����ٶ����Ƽ���Ķ�̬����
Vd=[x(4)-Kinematic(4)*dt x(4)+Kinematic(4)*dt x(5)-Kinematic(5)*dt x(5)+Kinematic(5)*dt x(6)-Kinematic(6)*dt x(6)+Kinematic(6)*dt];
% ���յ�Dynamic Window
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4)) max(Vtmp(:,5)) min(Vtmp(:,6))];