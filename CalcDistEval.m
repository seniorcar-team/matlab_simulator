%% �������̎Z�o�i���s�҂͌��݈ʒu�𗘗p�j
function [dist,ind]=CalcDistEval(x,Pedestrian,dist_max)
%���s�҂Ƃ̋����]���l���v�Z����֐�
for io=1:length(Pedestrian)
    disttmp(io)=norm(Pedestrian(io).tmp_position-x(1:2)');%�p�X�̈ʒu�Ə�Q���Ƃ̃m�����덷���v�Z
end
[dist_ob,ind] = min(disttmp);
dist = min([dist_max, dist_ob]);