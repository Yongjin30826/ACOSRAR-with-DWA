
function sol = CartToSpherical(position,model,VarMax,VarMin)

    % 获取笛卡尔坐标
    x = position.x;
    y = position.y;
    z = position.z;
    
    % 初始化解结构体
    sol.r = zeros(1, model.n);
    sol.psi = zeros(1, model.n);
    sol.phi = zeros(1, model.n);
    % Start location
    xs = model.start(1);
    ys = model.start(2);
    zs = model.start(3);
    
    
    % 第一个坐标
    sol.r(1) = sqrt((x(1) - xs)^2 + (y(1) - ys)^2 + (z(1) - zs)^2);
    if sol.r(1) > VarMax.r
        sol.r(1) = VarMax.r;
    end
    if sol.r(1) < VarMin.r
        sol.r(1) = VarMin.r;
    end
    
    sol.psi(1) = atan2(z(1) - zs, sqrt((x(1) - xs)^2 + (y(1) - ys)^2));
    if sol.psi(1) > VarMax.psi
        sol.psi(1) = VarMax.psi;
    end
    if sol.psi(1) < VarMin.psi
        sol.psi(1) = VarMin.psi;
    end
    
    sol.phi(1) = atan2(x(1) - xs,y(1) - ys);
    if sol.phi(1) > VarMax.phi
        sol.phi(1) = VarMax.phi;
    end
    if sol.phi(1) < VarMin.phi
        sol.phi(1) = VarMin.phi;
    end
    
    
% 转换笛卡尔坐标为球坐标
    for i = 2:model.n


        % 计算球坐标
        sol.r(i) = sqrt((x(i) - x(i-1))^2 + (y(i) - y(i-1))^2 + (z(i) - z(i-1))^2);
        if sol.r(i) > VarMax.r
           sol.r(i) = VarMax.r;
        end
        if sol.r(i) < VarMin.r
           sol.r(i) = VarMin.r;
        end

        sol.psi(i) = atan2(z(i) - z(i-1), sqrt((x(i) - x(i-1))^2 + (y(i) - y(i-1))^2));
        if sol.psi(i) > VarMax.psi
           sol.psi(i) = VarMax.psi;
        end
        if sol.psi(i) < VarMin.psi
           sol.psi(i) = VarMin.psi;
        end
        
        sol.phi(i) = atan2(x(i) - x(i-1),y(i) - y(i-1));
        if sol.phi(i) > VarMax.phi
            sol.phi(i) = VarMax.phi;
        end
        if sol.phi(i) < VarMin.phi
            sol.phi(i) = VarMin.phi;
        end
    end


end