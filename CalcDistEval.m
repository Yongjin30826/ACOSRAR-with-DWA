function [distStitic,distMove]=CalcDistEval(x,obsStatic,obsMove)
distStitic=50;
numObsStatic = length(obsStatic); % ��̬�ϰ�������
for io=1:numObsStatic
    disttmp=norm(obsStatic{io}.xyz(1:2)-x(1:2))-obsStatic{io}.R;
    % ���ϰ�����С�ľ���
    if distStitic>disttmp
        distStitic=disttmp;
    end
end
% �ϰ�����������޶�һ�����ֵ��������趨��һ��һ���켣û���ϰ����̫ռ����
if distStitic>=50
    distStitic=50;
end

distMove=50;
numObsMove = length(obsMove); % ��̬�ϰ�������
for io=1:numObsMove
    disttmp=norm(obsMove{io}.pos-x(1:3))-obsMove{io}.R;
    % ���ϰ�����С�ľ���
    if distMove>disttmp
        distMove=disttmp;
    end
end
% �ϰ�����������޶�һ�����ֵ��������趨��һ��һ���켣û���ϰ����̫ռ����
if distMove>=50
    distMove=50;
end

% dist=50;
% vecV = x(4:6);
% numObsStatic = length(obsStatic); % ��̬�ϰ�������
% for io=1:numObsStatic
%     vecDisTmp = x(1:3) - obsStatic{io}.xyz;
%     disttmp = (norm(vecDisTmp)-obsStatic{io}.R) * vecV'*vecDisTmp;
%     if size(disttmp,1)>1
%         1;
%     end
%     %     disttmp=norm(obsStatic{io}.xyz(1:2)-x(1:2)')-obsStatic{io}.R;
%     % ���ϰ�����С�ľ���
%     if dist>disttmp
%         dist=disttmp;
%     end
% end
% numObsMove = length(obsMove); % ��̬�ϰ�������
% for io=1:numObsMove
%     %     disttmp=norm(obsMove{io}.pos-x(1:3))-obsMove{io}.R;
%     vecDisTmp = x(1:3) - obsMove{io}.pos;
%     disttmp = (norm(vecDisTmp)-obsMove{io}.R) * vecV'*vecDisTmp;
%     if size(disttmp,1)>1
%         1;
%     end
%     % ���ϰ�����С�ľ���
%     if dist>disttmp
%         dist=disttmp;
%     end
% end

% �ϰ�����������޶�һ�����ֵ��������趨��һ��һ���켣û���ϰ����̫ռ����
% if dist>=50
%     dist=50;
% end