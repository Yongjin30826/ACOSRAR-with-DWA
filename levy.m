% Levy flight
% Coded by Hemanth Manjunatha on Nov 13 2015.

% Input parameters
% n     -> Number of steps 
% m     -> Number of Dimensions 
% beta  -> Power law index  U(0, 2] 
function [z]=levy(n,m)
    while(1)
        beta=2*rand;  % May be better to make it random in (0, 2] since it affects performance with small values mean large jumps
        if beta > 0.1 % beta can't be zero or even close to zero (e.g., a value like 1e-4 causes sigma_u to be NaN).
            break;
        end
    end
    
    num = gamma(1+beta)*sin(pi*beta/2); % used for Numerator 
    
    den = gamma((1+beta)/2)*beta*2^((beta-1)/2); % used for Denominator

    sigma_u = (num/den)^(1/beta);% Standard deviation
    
    u=random('Normal',0,sigma_u^2,n,m);
    
    v = random('Normal',0,1,n,m);

    z = u./(abs(v).^(1/beta));

end