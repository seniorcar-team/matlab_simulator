%% �ǂƂ̋������̌v�Z
function dist_wall=CalcDistObstacleEvalAll(traj,dist_max)
global wall_left;
global wall_right;
%�ǂƂ̋����]���l���v�Z����֐�
% dist_wall = dist_max;
dist_wall = 10;
for i = 1:length(traj(1,:))
        di_wall_right = traj(2,i) - wall_right;
        di_wall_left = wall_left - traj(2,i);
        di_wall = min(di_wall_right,di_wall_left);
%         dist_ob_set(io)=norm(di_ob);%�p�X�̈ʒu�Ə�Q���Ƃ̃m�����덷���v�Z
        if di_wall < dist_wall
            dist_wall = di_wall;
        end
end
% dist_ob = min(dist_ob_set);
% dist_ob = min([dist_max, dist_ob]);