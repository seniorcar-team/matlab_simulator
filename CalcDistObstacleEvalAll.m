%% 壁との距離項の計算
function dist_wall=CalcDistObstacleEvalAll(traj,dist_max)
global wall_left;
global wall_right;
%壁との距離評価値を計算する関数
% dist_wall = dist_max;
dist_wall = 10;
for i = 1:length(traj(1,:))
        di_wall_right = traj(2,i) - wall_right;
        di_wall_left = wall_left - traj(2,i);
        di_wall = min(di_wall_right,di_wall_left);
%         dist_ob_set(io)=norm(di_ob);%パスの位置と障害物とのノルム誤差を計算
        if di_wall < dist_wall
            dist_wall = di_wall;
        end
end
% dist_ob = min(dist_ob_set);
% dist_ob = min([dist_max, dist_ob]);