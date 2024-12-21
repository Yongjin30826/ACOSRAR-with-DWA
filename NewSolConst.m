% New solutions construction
function [NewAnt,Nfitx]= NewSolConst(ModelInfor, x, PopSize, Dimension, Probability, flag,zeta)
ell=RouletteWheelSelection(Probability);
StandardDeviation=zeta.*sum(abs(x(ell,:)-x))./(PopSize-1);  % Dimension *PopSize   
Step=StepSelection(ell,flag, Dimension);
NewAnt=x(ell,:)+StandardDeviation.*Step;
Nfitx= CostFunction(SphericalToCart(InitPos,model));      
end

function Step=StepSelection(ell,flag, Dimension)
flag=flag(ell,:);
Step=randn(1,Dimension).*(1-flag)+trnd(1, 1, Dimension).*flag;   %  (高斯变异+柯西变异）
% Gaussian randn(1,Dimension) ; Cauchy trnd(1, 1, Dimension)
end