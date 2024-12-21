function [headingxy,headingxz]=CalcHeadingEval(x,goal)

% 无人机运动朝向
thetaxy = toDegree(atan2(x(5),x(4))); % xy平面的朝向
thetaxz = toDegree(atan2(x(6),norm([x(4),x(5)]))); % xz平面的朝向
% 目标点的方位
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
% theta越小，评价越高
headingxy = 180 - targetThetaxy;
headingxz = 180 - targetThetaxz;

