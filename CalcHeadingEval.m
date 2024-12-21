function [headingxy,headingxz]=CalcHeadingEval(x,goal)

% ���˻��˶�����
thetaxy = toDegree(atan2(x(5),x(4))); % xyƽ��ĳ���
thetaxz = toDegree(atan2(x(6),norm([x(4),x(5)]))); % xzƽ��ĳ���
% Ŀ���ķ�λ
goalThetaxy = toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));
goalThetaxz = toDegree(atan2(goal(3)-x(3),norm([goal(1)-x(1),goal(2)-x(2)])));
if goalThetaxy > thetaxy
    targetThetaxy = goalThetaxy - thetaxy;% [deg]
else
    targetThetaxy = thetaxy - goalThetaxy;% [deg]
end
if goalThetaxz > thetaxz
    targetThetaxz = goalThetaxz - thetaxz;% [deg]
else
    targetThetaxz = thetaxz - goalThetaxz;% [deg]
end
% thetaԽС������Խ��
headingxy = 180 - targetThetaxy;
headingxz = 180 - targetThetaxz;

