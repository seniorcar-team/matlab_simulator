%% �ǂƂ̋������̌v�Z
function dist_ob=CalcDistObstacleEval(x,ob,dist_max)
%�ǂƂ̋����]���l���v�Z����֐�
for io=1:length(ob)
    di_ob(1)=ob(io,1)-x(1);
    di_ob(2)=ob(io,2)-x(2);
    dist_ob_set(io)=norm(di_ob);%�p�X�̈ʒu�Ə�Q���Ƃ̃m�����덷���v�Z
end
dist_ob = min(dist_ob_set);
dist_ob = min([dist_max, dist_ob]);