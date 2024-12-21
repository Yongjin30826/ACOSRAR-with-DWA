function Position_new = repair_z(Position,model)

H = model.H;
% 障碍物/威胁 坐标
threats = model.threats;
threat_num = size(threats,1);
zmax = model.zmax;
zmin = model.zmin;


% 定义 路径点 坐标
x_old = Position.x; % 修复之前的x坐标
y_old = Position.y; % 修复之前的y坐标
z_old = Position.z; % 修复之前的z坐标
% Start location
xs=model.start(1);
ys=model.start(2);
zs=model.start(3);

% Final location
xf=model.end(1);
yf=model.end(2);
zf=model.end(3);

x_all = [xs x_old xf];
y_all = [ys y_old yf];
z_all = [zs z_old zf];
N = size(x_all,2); 

% 绝对高度 = 相对高度（z_relative） + 地面高度（ground_level）
z_abs = zeros(1,N);
for i = 1:N
    z_abs(i) = z_all(i) + H(round(y_all(i)),round(x_all(i)));
end


for i = 1:threat_num
    threat = threats(i,:);
    threat_x = threat(1);
    threat_y = threat(2);
    threat_z = threat(3);
    threat_radius = threat(4);
    threat_height = threat(5);
    for j = 2:N-1      %  路径上的点（不包含两个端点 ）
        % 点在xy平面上的投影与圆心的距离
        distance = sqrt((x_all(j) - threat_x)^2 + (y_all(j) - threat_y)^2);   % 点和圆柱体投影之后到圆心的距离
        if distance > threat_radius + 1  % 大于 障碍物+无人机自身大小 视为无碰撞
            [];
        else  
            if z_abs(j) > threat_z + threat_height   % 无人机在障碍物上方
               [];
            else
                % 无人机在障碍物的下方
                znew_abs = (H(round(y_all(j)),round(x_all(j))) + zmax - threat_z - threat_height) * rand() + threat_z + threat_height;  % 生成一个新解
                z_abs(j) = znew_abs;
            end
            z_new = z_abs(j)- H(round(y_all(j)),round(x_all(j)));
            z_old(j-1)=z_new;
        end
    end
end
            

Position_new.x = x_old;
Position_new.y = y_old;
Position_new.z = z_old;   
end

