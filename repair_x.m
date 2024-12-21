function Position_new = repair_x(Position,model)

% 障碍物/威胁 坐标
x_threats = model.threats(:,1); % 障碍物/威胁   x坐标
y_threats = model.threats(:,2); % 障碍物/威胁   y坐标
r_threats = model.threats(:,4); % 障碍物/威胁   r半径
n_threats = size( model.threats,1);  % 障碍物/威胁 数量

% 定义 路径点 坐标
x_old = Position.x; % 修复之前的x坐标
y_old = Position.y; % 修复之前的y坐标
z_old = Position.z; % 修复之前的z坐标

obsx1=[];
obsx2=[];
% 循环计算每个 路径点 到每个 障碍物/威胁 的距离并执行相应操作

for i = 1:length(y_old)
    for j = 1:length(y_threats)
        distance = sqrt((x_old(i) - x_threats(j))^2 + (y_old(i) - y_threats(j))^2);
        
        if distance < r_threats(j)
            
            for k = 1:n_threats
                
                x1 = (r_threats(k)^2 - (y_old(i) - y_threats(k))^2 )^0.5+x_threats(k);   %  每个障碍物/威胁 右坐标
                if isreal(x1)
                    obsx1 = [obsx1, x1];
                end
                
                x2 = -(r_threats(k)^2 - (y_old(i) - y_threats(k))^2 )^0.5+x_threats(k);  % 每个障碍物/威胁 左坐标
                
                if isreal(x2)
                    obsx2 = [obsx2, x2];
                end
            end
            

            % 定义不可行区域的上下限
            infeasible_ranges = [obsx2;obsx1]';
            % 给定范围
            x_min = model.xmin;
            x_max = model.xmax;
            
            % 不可行区域数量
            n_infeasible = size(infeasible_ranges,1);  % 不可行区域数量
            
            % 随机生成一个不属于不可行区域的数
            
            % 生成大量的随机数
            num_samples = 200;
            random_numbers = linspace(x_min, x_max, num_samples)';
           
            % 创建一个逻辑矩阵，表示random_numbers中的元素是否在不可行区域内
            condition = false(size(random_numbers)); % 初始化为全假
            for aa = 1:n_infeasible % 遍历每个区域
                condition = condition | (random_numbers >= infeasible_ranges(aa,1) & random_numbers <= infeasible_ranges(aa,2)); % 更新逻辑矩阵
            end
            
            % 创建一个新的矩阵，只包含A中不在不可行区域内的元素
            feasible = random_numbers(~condition);
            random_index = randi(length(feasible));
            x_new = feasible(random_index);
            x_newsol= x_new;
            x_old(i) = x_newsol;      
        else
            [];
        end
    end  
end

Position_new.x = x_old;
Position_new.y = y_old;
Position_new.z = z_old;

end