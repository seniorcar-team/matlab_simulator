%% 壁との衝突予測(予測時間までの車両軌道が壁と衝突しないかの判定を行う)
function safety_wall=CalcWallCheck_margin_2(ob,traj)
global vehicle_width;
global vehicle_length;
global vehicle_front_length;
global vehicle_rear_length;
global safety_margin_front;
global safety_margin_rear;
global safety_margin_side;
global wall_right;
global wall_left;
safety_wall=1;
for i = 1:length(traj(1,:))
    if traj(2,i) < wall_left -...
        ((vehicle_front_length+safety_margin_front)^2 + (vehicle_width/2+safety_margin_side)^2)^0.5...
        && traj(2,i) > wall_right +...
        ((vehicle_front_length+safety_margin_front)^2 + (vehicle_width/2+safety_margin_side)^2)^0.5
        continue
    end
    if safety_wall == 1
        %　車両の4隅の座標の計算
        x_car(1) = traj(1,i)+(vehicle_front_length + safety_margin_front)*cos(traj(3,i))...
                            -(vehicle_width/2 + safety_margin_side)*sin(traj(3,i));
        y_car(1) = traj(2,i)+(vehicle_front_length + safety_margin_front)*sin(traj(3,i))...
                            +(vehicle_width/2 + safety_margin_side)*cos(traj(3,i));
        x_car(2) = traj(1,i)-(vehicle_rear_length + safety_margin_rear)*cos(traj(3,i))...
                            -(vehicle_width/2 + safety_margin_side)*sin(traj(3,i));
        y_car(2) = traj(2,i)-(vehicle_rear_length + safety_margin_rear)*sin(traj(3,i))...
                            +(vehicle_width/2 + safety_margin_side)*cos(traj(3,i));
        x_car(3) = traj(1,i)-(vehicle_rear_length + safety_margin_rear)*cos(traj(3,i))...
                            +(vehicle_width/2 + safety_margin_side)*sin(traj(3,i));
        y_car(3) = traj(2,i)-(vehicle_rear_length + safety_margin_rear)*sin(traj(3,i))...
                            -(vehicle_width/2 + safety_margin_side)*cos(traj(3,i));
        x_car(4) = traj(1,i)+(vehicle_front_length + safety_margin_front)*cos(traj(3,i))...
                            +(vehicle_width/2 + safety_margin_side)*sin(traj(3,i));
        y_car(4) = traj(2,i)+(vehicle_front_length + safety_margin_front)*sin(traj(3,i))...
                            -(vehicle_width/2 + safety_margin_side)*cos(traj(3,i));
        %　車両の4隅が壁に衝突するかの判定
        if ((y_car(1) > wall_left ) || (y_car(1) < wall_right ) || ...
                (y_car(2) > wall_left ) || (y_car(2) < wall_right ) ||...
                (y_car(3) > wall_left ) || (y_car(3) < wall_right ) ||...
                (y_car(4) > wall_left ) || (y_car(4) < wall_right ) )
                 safety_wall = 0;
                 break;
        end
    else
        break;
    end
end
