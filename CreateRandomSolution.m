%
% Create random paths (solutions)
% 

function sol=CreateRandomSolution(VarSize,VarMin,VarMax) 
    % Random path nodes
    sol.r=unifrnd(VarMin.r,VarMax.r,VarSize);
    sol.psi=unifrnd(VarMin.psi,VarMax.psi,VarSize);
    sol.phi=unifrnd(VarMin.phi,VarMax.phi,VarSize);
%       sol.r=Logistic_r(i,:).*(VarMax.r-VarMin.r)+VarMin.r;
%       sol.psi=Logistic_psi(i,:).*(VarMax.psi-VarMin.psi)+VarMin.psi;
%       sol.phi=Logistic_phi(i,:).*(VarMax.phi-VarMin.phi)+VarMin.phi;




end