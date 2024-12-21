function [u,trajDB]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obsStatic,obsMove)

% 动态窗口 [vxmin,vxmax,vymin,vymax,vzmin,vzmax]
Vr=CalcDynamicWindow(x,Kinematic);

% 评价函数的计算
[evalDB,trajDB]=Evaluation(x,Vr,goal,obsStatic,obsMove,Kinematic,evalParam);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0;0];return;
end

% % 各评价函数正则化
evalDB=NormalizeEval(evalDB);

% 最终评价函数的计算
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:7)*evalDB(id,:)'];
end
evalDB=[evalDB feval];
% 最优评价函数
[maxv,ind]=max(feval);
% 最优的速度和角速度
u=evalDB(ind,1:3)';