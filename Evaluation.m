function [evalDB,trajDB]=Evaluation(x,Vr,goal,obsStatic,obsMove,Kinematic,evalParam)

% 评价函数值
evalDB=[];
% 预测轨迹
trajDB=[];
for vx=Vr(1):Kinematic(7):Vr(2)
    for vy=Vr(3):Kinematic(8):Vr(4)
        for vz=Vr(5):Kinematic(9):Vr(6)
            % GenerateTrajectory轨迹推测
            % 得到 xt: 无人机向前运动后的预测位姿; traj: 当前时刻到预测时刻之间的轨迹
            % evalParam(end),前向模拟时间;
            [xt,traj]=GenerateTrajectory(x,vx,vy,vz,evalParam(end),Kinematic);
%             xt
            %% 各评价函数的计算
            % 当前无人机和目标点的朝向
            [headingxy,headingxz]=CalcHeadingEval(xt,goal);
            % 当前无人机和障碍物的距离
            [distStatic,distMove]=CalcDistEval(xt,obsStatic,obsMove);

evalDB=[evalDB;[vx vy vz headingxy,headingxz distStatic distMove]];
                trajDB=[trajDB;traj];
%             end
        end
    end
end