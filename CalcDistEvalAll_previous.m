%% dist関数の算出（歩行者は現在位置を利用）（既存手法の再現）
function dist_all=CalcDistEvalAll_previous(x,u,Pedestrian)
global wheel_base; %ホイールベース[m]
% global safety_margin_front;
% global safety_margin_rear;
global safety_margin_side;
global pedestrian_width;
global vehicle_width;
global wall_left; 
global wall_right; 

dist_max = 10; 
dist_ped = dist_max;
dist_wall_right = dist_max;
dist_wall_left = dist_max;

if u(2)~=0
    R = abs(wheel_base/tan(abs(u(2))));
else
    R=100000;
end

% 回転方向
if u(2) > 0
    sgn = 1;
    rotation = -1/2*pi;
else
    sgn = -1;
    rotation = 1/2*pi;
end
theta_senior_angle = Pi2Pi(x(3)+ rotation);
center_x = x(1) - R*cos(theta_senior_angle); % 中心座標
center_y = x(2) - R*sin(theta_senior_angle); % 中心座標           
center = [center_x center_y];
% center
% rad2deg(u(2))
%歩行者との距離評価値を計算する関数
    for io=1:length(Pedestrian)
        disttmp = Pedestrian(io).tmp_position - center;
        R_dash = norm(disttmp);
        if R_dash < R + vehicle_width/2 + safety_margin_side + pedestrian_width &&...
                R_dash > R - (vehicle_width/2 + safety_margin_side + pedestrian_width)
            theta_ped_angle = atan2(disttmp(2),disttmp(1));
            theta_angle = Pi2Pi360(sgn*Pi2Pi(theta_ped_angle - theta_senior_angle));
%             theta_angle
            dist_ped_io = R_dash*theta_angle;
%             R
%             R_dash
%             if io==1
%                 rad2deg(u(2))

%             dist_ped_io
%             io
%             end
            if dist_ped > dist_ped_io
                dist_ped = dist_ped_io;
            end
        end
    end
%     dist_ped
    di_wall_right = center(2) - wall_right;
    di_wall_left = wall_left - center(2);
    if abs(di_wall_right) < R + vehicle_width/2 + safety_margin_side
       x1 = center(1) + (-(wall_right-center(2))^2+(R+ vehicle_width/2 + safety_margin_side)^2)^0.5;
       x2 = center(1) - (-(wall_right-center(2))^2+(R+ vehicle_width/2 + safety_margin_side)^2)^0.5;
       theta_ped_angle_1 = atan2(wall_right-center(2),x1-center(1));
       theta_ped_angle_2 = atan2(wall_right-center(2),x2-center(1));
       theta_angle_1 = Pi2Pi360(sgn*Pi2Pi(theta_ped_angle_1 - theta_senior_angle));
       theta_angle_2 = Pi2Pi360(sgn*Pi2Pi(theta_ped_angle_2 - theta_senior_angle));
       dist_wall_right = min(theta_angle_1,theta_angle_2)*R;
%        dist_wall_right
    end
       
    if abs(di_wall_left) < R + vehicle_width/2 + safety_margin_side
       x1 = center(1) + (-(wall_left-center(2))^2+(R+ vehicle_width/2 + safety_margin_side)^2)^0.5;
       x2 = center(1) - (-(wall_left-center(2))^2+(R+ vehicle_width/2 + safety_margin_side)^2)^0.5;
       theta_ped_angle_1 = atan2(wall_left-center(2),x1-center(1));
       theta_ped_angle_2 = atan2(wall_left-center(2),x2-center(1));
       theta_angle_1 = Pi2Pi360(sgn*Pi2Pi(theta_ped_angle_1 - theta_senior_angle));
       theta_angle_2 = Pi2Pi360(sgn*Pi2Pi(theta_ped_angle_2 - theta_senior_angle));
       dist_wall_left = min(theta_angle_1,theta_angle_2)*R;
%        dist_wall_left
    end
    dist_wall = min(dist_wall_right,dist_wall_left);
    dist_all = min(dist_ped,dist_wall);
%     dist_all
