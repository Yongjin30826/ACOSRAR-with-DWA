function EvalDB=NormalizeEval(EvalDB)
EvalDB(:,4) = EvalDB(:,4) - min(EvalDB(:,4));
EvalDB(:,5) = EvalDB(:,5) - min(EvalDB(:,5));
EvalDB(:,6) = EvalDB(:,6) - min(EvalDB(:,6));
EvalDB(:,7) = EvalDB(:,7) - min(EvalDB(:,7));
if sum(EvalDB(:,4))~=0
    EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~=0
    EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
end
if sum(EvalDB(:,6))~=0
    EvalDB(:,6)=EvalDB(:,6)/sum(EvalDB(:,6));
end
if sum(EvalDB(:,7))~=0
    EvalDB(:,7)=EvalDB(:,7)/sum(EvalDB(:,7));
end