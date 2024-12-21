function [distStitic,distMove]=CalcDistEval(x,obsStatic,obsMove)
distStitic=50;
numObsStatic = length(obsStatic); % 静态障碍物数量
for io=1:numObsStatic
    disttmp=norm(obsStatic{io}.xyz(1:2)-x(1:2))-obsStatic{io}.R;
    % 离障碍物最小的距离
    if distStitic>disttmp
        distStitic=disttmp;
    end
end
% 障碍物距离评价限定一个最大值，如果不设定，一旦一条轨迹没有障碍物，将太占比重
if distStitic>=50
    distStitic=50;
end

distMove=50;
numObsMove = length(obsMove); % 静态障碍物数量
for io=1:numObsMove
    disttmp=norm(obsMove{io}.pos-x(1:3))-obsMove{io}.R;
    % 离障碍物最小的距离
    if distMove>disttmp
        distMove=disttmp;
    end
end
% 障碍物距离评价限定一个最大值，如果不设定，一旦一条轨迹没有障碍物，将太占比重
if distMove>=50
    distMove=50;
end

% dist=50;
% vecV = x(4:6);
% numObsStatic = length(obsStatic); % 静态障碍物数量
% for io=1:numObsStatic
%     vecDisTmp = x(1:3) - obsStatic{io}.xyz;
%     disttmp = (norm(vecDisTmp)-obsStatic{io}.R) * vecV'*vecDisTmp;
%     if size(disttmp,1)>1
%         1;
%     end
%     %     disttmp=norm(obsStatic{io}.xyz(1:2)-x(1:2)')-obsStatic{io}.R;
%     % 离障碍物最小的距离
%     if dist>disttmp
%         dist=disttmp;
%     end
% end
% numObsMove = length(obsMove); % 静态障碍物数量
% for io=1:numObsMove
%     %     disttmp=norm(obsMove{io}.pos-x(1:3))-obsMove{io}.R;
%     vecDisTmp = x(1:3) - obsMove{io}.pos;
%     disttmp = (norm(vecDisTmp)-obsMove{io}.R) * vecV'*vecDisTmp;
%     if size(disttmp,1)>1
%         1;
%     end
%     % 离障碍物最小的距离
%     if dist>disttmp
%         dist=disttmp;
%     end
% end

% 障碍物距离评价限定一个最大值，如果不设定，一旦一条轨迹没有障碍物，将太占比重
% if dist>=50
%     dist=50;
% end