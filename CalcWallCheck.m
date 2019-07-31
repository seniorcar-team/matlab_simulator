%% •Ç‚Æ‚ÌÕ“Ë—\‘ª(—\‘ªŠÔ‚Ü‚Å‚ÌÔ—¼‹O“¹‚ª•Ç‚ÆÕ“Ë‚µ‚È‚¢‚©‚Ì”»’è‚ğs‚¤)
function safety_wall=CalcWallCheck_margin_2(traj)
global vehicle_width;
global vehicle_length;
global vehicle_front_length;
global vehicle_rear_length;
global safety_margin_front;
global safety_margin_rear;
global safety_margin_side;
global wall_right;
global wall_left;
global road_width;
global road_length;
safety_wall=1;
turning_radius = 0.9/tan(deg2rad(35));
turning_radius_2 = 1.9;%ÅŠO‘¤‚ª•`‚­‰~‚Ì”¼Œa
for i = 1:length(traj(1,:))
    if traj(2,i) < wall_left -...
        ((vehicle_front_length)^2 + (vehicle_width/2)^2)^0.5...
        && traj(2,i) > wall_right +...
        ((vehicle_front_length)^2 + (vehicle_width/2)^2)^0.5
        continue
    end
    if safety_wall == 1
        %@Ô—¼‚Ì4‹÷‚ÌÀ•W‚ÌŒvZ
        x_car(1) = traj(1,i)+(vehicle_front_length )*cos(traj(3,i))...
                            -(vehicle_width/2 )*sin(traj(3,i));
        y_car(1) = traj(2,i)+(vehicle_front_length )*sin(traj(3,i))...
                            +(vehicle_width/2 )*cos(traj(3,i));
        x_car(2) = traj(1,i)-(vehicle_rear_length)*cos(traj(3,i))...
                            -(vehicle_width/2 )*sin(traj(3,i));
        y_car(2) = traj(2,i)-(vehicle_rear_length )*sin(traj(3,i))...
                            +(vehicle_width/2 )*cos(traj(3,i));
        x_car(3) = traj(1,i)-(vehicle_rear_length )*cos(traj(3,i))...
                            +(vehicle_width/2 )*sin(traj(3,i));
        y_car(3) = traj(2,i)-(vehicle_rear_length )*sin(traj(3,i))...
                            -(vehicle_width/2 )*cos(traj(3,i));
        x_car(4) = traj(1,i)+(vehicle_front_length )*cos(traj(3,i))...
                            +(vehicle_width/2 )*sin(traj(3,i));
        y_car(4) = traj(2,i)+(vehicle_front_length )*sin(traj(3,i))...
                            -(vehicle_width/2 )*cos(traj(3,i));
                        
        %@Ô—¼‚Ì4‹÷‚ª•Ç‚ÉÕ“Ë‚·‚é‚©‚Ì”»’è
        turning_center_counter_clockwise_y_position = traj(2,i) + turning_radius*sin(traj(3,i)+pi/2);
        turning_center_clockwise_y_position = traj(2,i) + turning_radius*sin(traj(3,i)-pi/2);

        collision_check_point = zeros(2,1);
        collision_check_point(1) = turning_center_counter_clockwise_y_position - turning_radius_2;
        collision_check_point(2) = turning_center_clockwise_y_position + turning_radius_2;
        if traj(2,i)<0 && collision_check_point(1)< -road_width/2
            safety_wall = 0;
            %disp('circle');
            return;
        elseif traj(2,i)>=0 && collision_check_point(2)> road_width/2
            safety_wall = 0;
            %disp('circle');
            return;
        end
        if max(abs(y_car(:,2)))>road_width/2
            safety_wall = 0;
            %disp('4point');
            %disp('!');
            return;
        end
    else
        break;
    end
end

