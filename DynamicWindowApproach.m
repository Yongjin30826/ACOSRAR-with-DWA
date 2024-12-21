function [u,trajDB]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obsStatic,obsMove)

% ��̬���� [vxmin,vxmax,vymin,vymax,vzmin,vzmax]
Vr=CalcDynamicWindow(x,Kinematic);

% ���ۺ����ļ���
[evalDB,trajDB]=Evaluation(x,Vr,goal,obsStatic,obsMove,Kinematic,evalParam);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0;0];return;
end

% % �����ۺ�������
evalDB=NormalizeEval(evalDB);

% �������ۺ����ļ���
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:7)*evalDB(id,:)'];
end
evalDB=[evalDB feval];
% �������ۺ���
[maxv,ind]=max(feval);
% ���ŵ��ٶȺͽ��ٶ�
u=evalDB(ind,1:3)';