function [u,trajDB,evalDB,ind] = ...
        DynamicWindowApproach_CSC(x,Kinematic,goal,Pedestrian,PARAM)
ind = 0;
%Dynamic Window[vmin,vmax,��min,��max]�̍쐬
Vr=CalcDynamicWindow(x,Kinematic);

[evalDB,trajDB]=Evaluation_DWA_CSC(x,Vr,goal,Kinematic,PARAM,Pedestrian);%���̊֐��͌�ō��2018/10/12.16:23

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

if ~any(evalDB(:,3))
   if(x(4)~=0)
%        disp('no control value4');
       u=[Vr(1);x(5)];return;
%        pause
   end
%    pause
end
%�e�]���֐��̐��K��
evalDB=NormalizeEval_DWA_CSC(evalDB);%���̊֐��͌�ō��2018/10/12.16:23

%�ŏI�]���l�̌v�Z
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;PARAM(1:5)*evalDB(id,3:7)'];
end
evalDB=[evalDB feval];
% disp(evalDB);
[~,ind]=max(feval);%�ł��]���l���傫�����͒l�̃C���f�b�N�X���v�Z
uu=evalDB(ind,1:2)';%�]���l���������͒l(���x�C���Ǌp)��Ԃ�
if uu(1) <= 0.1
    uu(1)=uu(1);
    uu(2)= x(5);
    u = uu;
%     disp('hello');
else
%     disp(evalDB(ind,:));
    u=uu;
end