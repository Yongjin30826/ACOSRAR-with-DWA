function [evalDB,trajDB]=Evaluation(x,Vr,goal,obsStatic,obsMove,Kinematic,evalParam)

% ���ۺ���ֵ
evalDB=[];
% Ԥ��켣
trajDB=[];
for vx=Vr(1):Kinematic(7):Vr(2)
    for vy=Vr(3):Kinematic(8):Vr(4)
        for vz=Vr(5):Kinematic(9):Vr(6)
            % GenerateTrajectory�켣�Ʋ�
            % �õ� xt: ���˻���ǰ�˶����Ԥ��λ��; traj: ��ǰʱ�̵�Ԥ��ʱ��֮��Ĺ켣
            % evalParam(end),ǰ��ģ��ʱ��;
            [xt,traj]=GenerateTrajectory(x,vx,vy,vz,evalParam(end),Kinematic);
%             xt
            %% �����ۺ����ļ���
            % ��ǰ���˻���Ŀ���ĳ���
            [headingxy,headingxz]=CalcHeadingEval(xt,goal);
            % ��ǰ���˻����ϰ���ľ���
            [distStatic,distMove]=CalcDistEval(xt,obsStatic,obsMove);

evalDB=[evalDB;[vx vy vz headingxy,headingxz distStatic distMove]];
                trajDB=[trajDB;traj];
%             end
        end
    end
end