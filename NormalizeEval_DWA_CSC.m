function EvalDB=NormalizeEval_DWA_CSC(EvalDB)
%�]���l�𐳋K������֐�

for i =3:length(EvalDB(1,:))
    if max(EvalDB(:,i))~=0
        EvalDB(:,i)=EvalDB(:,i)/max(EvalDB(:,i));
    end
%     disp(max(EvalDB(:,i)));
end
