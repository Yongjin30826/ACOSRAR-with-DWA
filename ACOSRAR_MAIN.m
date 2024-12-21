% Revised: 12.2024
% This is ACOSRAR to optimize problems of UAV path planning in the 3D environment, based on the paper 
% "Ben Niu, Yongjin Wang, Jing Liu, and Gabriel Xiao-Guang Yue.Path Planning for Unmanned Aerial Vehicles in Complex 
%  Environment Based on an Improved Continuous Ant Colony Optimisation. Computers and Electrical Engineering"
clc;
clear;
close all;

% Number of runs
model = CreateModel(); % Create search map and parameters
CostFunction=@(x) MyCost(x,model);    % Cost Function
nVar=model.n;       % Number of Decision Variables = searching dimension of ACO = number of path nodes
VarSize=[1 nVar];   % Size of Decision Variables Matrix
numRuns = 1; 
MaxIt= 2;
allBestCosts = zeros(numRuns, MaxIt);

for run = 1: numRuns

%% Problem Definition

% Lower and upper Bounds of particles (Variables)
VarMin.x=model.xmin;
VarMax.x=model.xmax;
VarMin.y=model.ymin;
VarMax.y=model.ymax;
VarMin.z=model.zmin;
VarMax.z=model.zmax;

VarMax.r=4*norm(model.start-model.end)/nVar;
VarMin.r=0;

% Inclination (elevation)
AngleRange = pi/3; % Limit the angle range for better solutions
VarMin.psi=-AngleRange;
VarMax.psi=AngleRange;


% Azimuth
% Determine the angle of vector connecting the start and end points
dirVector = model.end - model.start;
phi0 = atan2(dirVector(2),dirVector(1));
VarMin.phi=phi0 - AngleRange;
VarMax.phi=phi0 + AngleRange;


%% ACO Parameters

nPop=500;           % Population Size (Swarm Size)
zeta=0.6;           % 用于高斯变异
q=0.2;              % 用于计算粒子权重
n_Ants=nPop;
it = 1;

%% 自适应修复方法的参数

n_Oper= 3; % 3种修复方法
win_size=10;  % 在概率更新之前的窗口大小(每个win_size进行依次选择概率更新)
n_win=2;  % 如果在过去的 n_win 个 win_size 大小窗口中策略没有对解进行提高或者改进，则重新初始化选择概率（每种修复方法的选择概率相同）
Experience_Oper=ones(1, n_Oper);    % 三种修复方法的经验值
NoImprove=0;

%% Initialization

% Create Empty Particle Structure
empty_particle.Position=[];
empty_particle.Cost=[];
GlobalBest.Cost=inf;

% Create an empty Particles Matrix, each particle is a solution (searching path)
particle=repmat(empty_particle,nPop,1);

% 初始化种群，初始化这里进行第一次修复，保证路径是可行的

isInit = false;
while (~isInit)
    disp("Initialising...");
    for i=1:nPop
        
        % 初始化位置
        particle(i).Position=CreateRandomSolution(VarSize,VarMin,VarMax);
        
        % 修复位置
        position_old = SphericalToCart(particle(i).Position,model);
        position_new = repair_y(position_old,model);
        
        % 将 笛卡尔 转换 球坐标
        particle(i).Position = CartToSpherical(position_new,model,VarMax,VarMin);
        
        % 成本
        particle(i).Cost= CostFunction(SphericalToCart(particle(i).Position,model));
        
        % Update Global Best
        if particle(i).Cost < GlobalBest.Cost
            GlobalBest.Position=particle(i).Position;
            GlobalBest.Cost=particle(i).Cost;
            isInit = true;
        end
    end
end


% 对初始化的种群进行排序
costValues = [particle.Cost];
[sortedCost, indecies ] = sort( costValues );
particle = particle(indecies);
GlobalBestCost=particle(1).Cost;

% 构建新解
empty_Newparticle.Position.r = zeros(1, nVar);
empty_Newparticle.Position.psi = zeros(1, nVar);
empty_Newparticle.Position.phi = zeros(1, nVar);
empty_Newparticle.Cost = [];
Newparticle= repmat(empty_Newparticle,nPop,1);

SolutionWeights=1/(q*nPop*sqrt(2*pi))*exp(-0.5*(((1:nPop)-1)/(q*nPop)).^2);   % 公式（22) 计算权重wi
Probability=SolutionWeights./sum(SolutionWeights);                            % 公式（21）计算每个粒子被选中的概率

BestCost=zeros(MaxIt,1);
q0=0.5;

%% ACO Main Loop
while it < MaxIt*nPop
    
    current_iter = (it-1)/500+1;  %1,2,3,4...
    BestCost(current_iter) = GlobalBestCost;    % BestCost 记录了每一个迭代结束后的最优解
    
    
    if mod(current_iter,win_size)==1        % iter 每满足一个Nwin之后，就要进行 p_r的更新（注意：经验值每次迭代都要更新，概率值则在预定窗口Nwin大小后进行更新）
        Prob_Oper=Experience_Oper/(sum(Experience_Oper)+realmin);  % realmin是处理非常小的浮点数值时，以确定可以接受的最小阈值。这可以用来避免舍入误差或处理接近零的数值
        Experience_Oper=ones(1, n_Oper);
    end
    moving_number = RouletteWheelSelection(Prob_Oper);   % 第一次时，当输入Prob_Oper=[0.333,0.333,0.333]时，随机数落在0-0.333,0.333-0.666,0.666-1.flag_moving分别是1，2，3 
    old_fitx = GlobalBestCost;
    
    % 为新解赋值
    for i = 1:nPop
        
        % r部分
        
        q1 = rand;  % 生成 [0, 1] 区间内的随机变量
        if q1 <= q0
            % 执行操作 argmax
            ell_r=1;
        else
            % 执行原来操作
            ell_r=RouletteWheelSelection(Probability);
        end
        
        all_r_values_cell = arrayfun(@(p) p.Position.r, particle, 'UniformOutput', false);      % 提取全部Position里边的r
        all_r_values = vertcat(all_r_values_cell{:});                                           % 将全部的r构成矩阵
        StandardDeviation_r=zeta.*sum(abs(particle(ell_r).Position.r-all_r_values))./(nPop-1);
        
        flag = randi([0,1]);  %  高斯变异和Levy的步长选择
        
        if flag == 0
            Step_r = randn(1,nVar);
        else
            Step_r = levy(1, nVar);
        end
        
        Newparticle(i).Position.r = particle(ell_r).Position.r+StandardDeviation_r.*Step_r;           % 使用高斯变异(连续ACO)
        
        % r 边界限制
        Newparticle(i).Position.r = max( Newparticle(i).Position.r,VarMin.r);
        Newparticle(i).Position.r = min( Newparticle(i).Position.r,VarMax.r);
        
        
        % psi 部分
        
        q2 = rand;  % 生成 [0, 1] 区间内的随机变量
        if q2 <= q0
            % 执行操作 argmax
            ell_psi=1;
        else
            
            ell_psi=RouletteWheelSelection(Probability);
        end
        
        all_psi_values_cell = arrayfun(@(p) p.Position.psi, particle, 'UniformOutput', false);      % 提取全部Position里边的psi
        all_psi_values = vertcat(all_psi_values_cell{:});                                           % 将全部的psi构成矩阵
        StandardDeviation_psi=zeta.*sum(abs(particle(ell_psi).Position.psi-all_psi_values))./(nPop-1);
        
        flag = randi([0,1]);  %  高斯变异和Levy的步长选择
        
        if flag == 0
            Step_psi = randn(1,nVar);
        else
            Step_psi = levy(1, nVar);
        end
        
        Newparticle(i).Position.psi = particle(ell_psi).Position.psi+StandardDeviation_psi.*Step_psi;
        
        
        % psi 边界限制
        Newparticle(i).Position.psi = max(Newparticle(i).Position.psi,VarMin.psi);
        Newparticle(i).Position.psi = min(Newparticle(i).Position.psi,VarMax.psi);
        
        
        
        % phi 部分
        q3 = rand;
        if q3 <= q0
            % 执行操作 argmax
            ell_phi=1;
        else
            ell_phi=RouletteWheelSelection(Probability);
        end
        
        all_phi_values_cell = arrayfun(@(p) p.Position.phi, particle, 'UniformOutput', false);      % 提取全部Position里边的psi
        all_phi_values = vertcat(all_phi_values_cell{:});                                           % 将全部的psi构成矩阵
        StandardDeviation_phi=zeta.*sum(abs(particle(ell_phi).Position.phi-all_phi_values))./(nPop-1);
        
        flag = randi([0,1]);  %  高斯变异和Levy的步长选择
        
        if flag == 0
            Step_phi = randn(1,nVar);
        else
            Step_phi = levy(1, nVar);
        end
        
        Newparticle(i).Position.phi = particle(ell_phi).Position.phi+StandardDeviation_phi.*Step_phi;
        
        % psi 边界限制
        Newparticle(i).Position.phi = max(Newparticle(i).Position.phi,VarMin.phi);
        Newparticle(i).Position.phi = min(Newparticle(i).Position.phi,VarMax.phi);
        
        %% 修复路径
        Position_old = SphericalToCart(Newparticle(i).Position,model);
        
        if moving_number == 1
            Position_new = repair_x(Position_old,model);
        elseif moving_number == 2
            Position_new = repair_y(Position_old,model);
        elseif moving_number == 3
            Position_new = repair_z(Position_old,model);
        end
        
        % 将 笛卡尔 转换 球坐标
        Newparticle(i).Position = CartToSpherical(Position_new,model,VarMax,VarMin);
        % B = SphericalToCart(Newparticle(i).Position,model);
        % 计算适应度值
        Newparticle(i).Cost=CostFunction(SphericalToCart(Newparticle(i).Position,model));
        
    end
    
    % 评估
    allSwarm = [particle; Newparticle];   % 两个种群进行合并
    cost_Allvalues = [allSwarm.Cost];
    [sorted_AllCost, all_indecies ] = sort( cost_Allvalues );
    allSwarm = allSwarm(all_indecies);
    
    % 筛选
    particle = allSwarm(1:nPop,:);   % 种群筛选到原规模大小
    
    % 更新每轮循环最优解
    Spherical_BestPosition = particle(1).Position;    %  最好的解在particle里，球坐标
    GlobalBestCost = particle(1).Cost;
    
    
    % 如果解的质量没有提高
    if old_fitx <= GlobalBestCost
        NoImprove=NoImprove+1;
    else
        NoImprove=0;
    end
    
    if NoImprove <= n_win*win_size    % 经过一次迭代就更新依次 经验值 
        Experience_Oper(moving_number)=Experience_Oper(moving_number)+abs((old_fitx-GlobalBestCost));  % 更新经验值
    else
        Experience_Oper=ones(1, n_Oper);      % 超出m个Nwin没有提高，则将E和p重新进行初始化
        Prob_Oper=Experience_Oper/(sum(Experience_Oper)+realmin);
    end
    
    it= it+nPop;
    allBestCosts(run, current_iter) = GlobalBestCost;
    % 每次迭代信息
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(GlobalBestCost)  ' Best Position.r = ' num2str(Spherical_BestPosition.r)   ' Best Position.spi = ' num2str(Spherical_BestPosition.psi)   ' Best Position.phi = ' num2str(Spherical_BestPosition.phi)]);
end

end


%% Plot results
% Best solution
BestPosition = SphericalToCart(Spherical_BestPosition,model);
disp("Best solution...");
BestPosition;
smooth = 0.95;
PlotSolution(BestPosition,model,smooth);

% Best cost
figure;
plot(BestCost,'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
save dataACO.mat