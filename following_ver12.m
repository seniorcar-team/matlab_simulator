function following_input = following_ver12(x,goal,Pedestrian,predict_obserbable_pedestrian_position)
%将来位置を使わない
%ver4では単純に最も距離が近いものに追従
%ver5では距離におうじて単純に加減速を行っていたものを，ポテンシャルベースのものに変更,距離をセニアカーの前端基準に変更
dt_sim = 1/10;
dt=1/10;
x_seniorcar = [x(1) x(2)];
x_seniorcar_front = [x(1)+0.9*cos(x(3)),x(2)+0.9*sin(x(3))];
speed_seniorcar = x(4);
min_turning_radius = x(4)^2/0.5;
%v_seniorcar = [x(4)*cos(x(3)),x(4)*sin(x(3))];
steer_angle = x(3);
goal_position = [goal(1),goal(2)];
r_goal = goal_position - x_seniorcar;
%ped_num = length(Pedestrian);
target_num = 0;
is_following = 0;
Evaluation_value = 0;
max_acceleration = 1.5;
%alfa = 3;%どのパラメータを重視するか？適当に調整
%beta = 2;
%ganma = 1;
%distance_criterion = 3.0;
%angle_criterion = deg2rad(45);
%speed_criterion = 6.0/3.6;
%safe_distance = 1.0 * (x(4)/(6.0/3.6))^2 + x(4)*dt_sim+0.9;
%margin = 0.1;%追従安定時の安全距離に対してさらにどの程度離れるか．

%max_speed = 6.0/3.6;
%delta_0 = 1.0;
%delta_1 = 1.5;
%delta_2 = 2.0;
observable_pedestrian_ind_array = find(~isnan(predict_obserbable_pedestrian_position(:,3)));
ped_num = length(observable_pedestrian_ind_array);

for i = 1:ped_num
    %r_seniorcar_to_Pedestrian = Pedestrian(predict_obserbable_pedestrian_position(observable_pedestrian_ind_array(i),1)).tmp_position-x_seniorcar;
    
    
        
    r_seniorcar_front_to_Pedestrian = Pedestrian(predict_obserbable_pedestrian_position(observable_pedestrian_ind_array(i),1)).tmp_position -x_seniorcar_front;
    d_seniorcar_to_Pedestrian = norm(r_seniorcar_front_to_Pedestrian);
    angle = atan2(r_seniorcar_front_to_Pedestrian(2),r_seniorcar_front_to_Pedestrian(1));
    if x(2) + 0.9*sin(x(3)) + 0.325*sin(x(3)+pi/2) > 1.5
        if angle > x(3)
            continue;
        end
    end
    if x(2) + 0.9*sin(x(3)) + 0.325*sin(x(3)-pi/2) < -1.5
        if angle < x(3)
            continue;
        end
    end
    %angle_threshold = min([0.003*d_seniorcar_to_Pedestrian/0.13,deg2rad(30)]);
    if abs(norm(Pedestrian(predict_obserbable_pedestrian_position(observable_pedestrian_ind_array(i),1)).tmp_position-x_seniorcar)/min_turning_radius)<=1
        angle_threthold = abs(asin(norm(Pedestrian(predict_obserbable_pedestrian_position(observable_pedestrian_ind_array(i),1)).tmp_position-x_seniorcar)/min_turning_radius));
    else
        angle_threthold = deg2rad(30);
    end
    if abs(angle) > angle_threthold
        continue;
    end
    if d_seniorcar_to_Pedestrian > 3
        continue;
    else
        angle_difference = abs(atan2(r_goal(2),r_goal(1)) - atan2(Pedestrian(predict_obserbable_pedestrian_position(observable_pedestrian_ind_array(i),1)).velocity(2),Pedestrian(predict_obserbable_pedestrian_position(observable_pedestrian_ind_array(i),1)).velocity(1)));
        if angle_difference > deg2rad(35)
            continue;
        end
        %speed_difference = abs(speed_seniorcar - norm(Pedestrian(predict_obserbable_pedestrian_position(observable_pedestrian_ind_array(i),1)).velocity));
        %if norm(speed_difference) > 0.5
         %   continue
        %end
        if norm(Pedestrian(predict_obserbable_pedestrian_position(observable_pedestrian_ind_array(i),1)).velocity)>6.0/3.6
            continue
        end
        
        Evaluation_function = 1 / d_seniorcar_to_Pedestrian;
        
        if Evaluation_value < Evaluation_function
            Evaluation_value = Evaluation_function;
            target_num = predict_obserbable_pedestrian_position(observable_pedestrian_ind_array(i),1);
            is_following = 1;
        end
        
    end
    
end

if is_following == 1
    
    %　現在の歩行者位置に対する地点追従制御を行う．pure pursuite
    target_position = [Pedestrian(target_num).tmp_position(1),...
                       Pedestrian(target_num).tmp_position(2)];
    if target_position(2) > 1.2
        target_position(2) = target_position(2) - 0.3;
    elseif target_position(2) < -1.2
        target_position(2) = target_position(2) + 0.3;
    end
    r_target_position = target_position - x_seniorcar;
    r_target_position_from_front = target_position - x_seniorcar_front;
    distance = norm(r_target_position_from_front);
    direction= atan2(r_target_position(2),r_target_position(1));
    angle_difference_2 = direction -x(3);
    if angle_difference_2 == 0
        turning_radius = 10000;
    else
        turning_radius = norm(r_target_position)/(2*sin(angle_difference_2));
    end
    %{
    if distance > 2
        speed_seniorcar = min([x(4)+1.5*dt/2,1.666,x(4)*1,1]);
    elseif distance<=2 && distance>1.5
        speed_seniorcar = x(4);
    elseif distance<=1.5 && distance>1.0
        speed_seniorcar = x(4)*0.95;
    elseif distance<=1.0 && distance>0
        speed_seniorcar = max([0,x(4)-1.5*dt/2]);
    else
        speed_seniorcar = 0;
    end
    
    omega = norm(v_seniorcar)/turning_radius;
    if speed_seniorcar == 0
        steer_angle = 0;
    else
        steer_angle = asin(0.9*omega/speed_seniorcar);
    end
    %}
    %e_target_position = r_target_position/norm(r_target_position);
    %速度は，追従対象歩行者の速度に近づけるように処理
    if distance < 1.5 && distance >1.0 
        speed_seniorcar = max([speed_seniorcar-0.5*dt/2,min([speed_seniorcar+0.5*dt/2,norm(Pedestrian(target_num).velocity)])]);
    elseif distance >= 1.5
        speed_seniorcar = max([speed_seniorcar-0.5*dt/2,min([speed_seniorcar+0.5*dt/2,1.1*norm(Pedestrian(target_num).velocity)])]);
    else
        speed_seniorcar = max([speed_seniorcar-0.5*dt/2,min([speed_seniorcar+0.5*dt/2,0.8*norm(Pedestrian(target_num).velocity)])]);
    end
    
    %omega = speed_seniorcar/turning_radius;
    if abs(speed_seniorcar^2/turning_radius) > 0.5
        if turning_radius  > 0
            turning_radius = speed_seniorcar^2*2;
        else
            turning_radius = - speed_seniorcar^2*2;
        end
    end
            
    if speed_seniorcar == 0
        steer_angle = 0;
    else
        steer_angle = atan(0.9/turning_radius);
    end
    
    if angle_difference_2 == 0%ここの処理を追加する
        steer_angle =0;
    end
    steer_angle = min([x(5) + deg2rad(50)*0.1 ,max([steer_angle,x(5) - deg2rad(50)* 0.1])]);%機械的な制約
        
end
steer_angle = max([-toRadian(35),min([steer_angle,toRadian(35)])]);
following_input = [speed_seniorcar steer_angle is_following];





