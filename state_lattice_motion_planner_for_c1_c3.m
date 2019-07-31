function path_array = state_lattice_motion_planner_for_c1_c3(x,existence_prohability_distribution_all,goal,predict_obserbable_pedestrian_position,time,Pedestrian)
path_array = NaN;
vel_str = num2str(min([16,max([1,round(x(4),1)*10])]));
k0 = [-deg2rad(30),-deg2rad(25),-deg2rad(20),-deg2rad(15),-deg2rad(10),-deg2rad(5),0,...
        deg2rad(5),deg2rad(10),deg2rad(15),deg2rad(20),deg2rad(25),deg2rad(30)];
A = abs(k0-x(5));
[~,ind] = min(A);
nearest_steer_angle = k0(ind);
steer_str = num2str(nearest_steer_angle);

file_name = ['look_up_table_nb_',vel_str,'_',steer_str];
lookuptable = load(file_name);

lookuptable  = sortrows(lookuptable,[1,2,3],'descend');
table_length = size(lookuptable,1);
r_goal = goal(1:2)-x(1:2)';
goal_direction = atan2(r_goal(2),r_goal(1));
lookuptable_plus = zeros(table_length,3);
lookuptable = [lookuptable,lookuptable_plus];
%size(lookuptable)
observable_pedestrian_ind = find(predict_obserbable_pedestrian_position(:,3)==time);
observable_pedestrian_num = length(observable_pedestrian_ind);
observable_pedestrian_mat = predict_obserbable_pedestrian_position(predict_obserbable_pedestrian_position(:,3)==time,:);
observable_pedestrian_direction = zeros(observable_pedestrian_num,1);
for iop = 1:observable_pedestrian_num
    if abs(atan2(Pedestrian(observable_pedestrian_ind(iop)).velocity(2),Pedestrian(observable_pedestrian_ind(iop)).velocity(1)))<pi/2
        observable_pedestrian_direction(iop) = 1;
    else
        observable_pedestrian_direction(iop) = 0;    
    end
end
if size(observable_pedestrian_ind,1) == 1
    observable_pedestrian_ind = observable_pedestrian_ind';
end
observable_pedestrian_mat =[observable_pedestrian_mat,observable_pedestrian_direction,observable_pedestrian_ind];
observable_pedestrian_mat = observable_pedestrian_mat(observable_pedestrian_mat(:,4)==0,:);%逆方向の奴だけ
observable_pedestrian_mat = observable_pedestrian_mat(observable_pedestrian_mat(:,2)<4,:);
observable_pedestrian_number = size(observable_pedestrian_mat,1);
direction_to_ped_opposit = zeros(observable_pedestrian_number,1);
for iop = 1:observable_pedestrian_number
    [r_x,r_y] = pol2cart(observable_pedestrian_mat(iop,1),observable_pedestrian_mat(iop,2));
    r_x = r_x +0.9;
    direction_to_ped_opposit(iop) = atan2(r_y,r_x);
end
for itable = 1:table_length
    r_last_state_to_goal = r_goal(1:2)-lookuptable(itable,1:2);
    direction_last_state_to_goal = atan2(r_last_state_to_goal(2),r_last_state_to_goal(1));
    lookuptable(itable,9:10) = [1.1-abs(1.1 - max([min([(x(4) + lookuptable(itable,6)*1.4),1.6]),0]))...
        ,pi-abs(pi_2_pi(goal_direction -(x(3) + atan2(lookuptable(itable,2),lookuptable(itable,1)))))...
        ];
end
%plot(lookuptable(:,1),lookuptable(:,2));hold on;
max_speed_eval = max(lookuptable(:,9));
max_direction_eval = max(lookuptable(:,10));
A_direc = direction_to_ped_opposit(direction_to_ped_opposit<0);
B_direc = direction_to_ped_opposit(direction_to_ped_opposit>=0);
plus_min_direc = min(B_direc);
minus_min_direc = min(abs(A_direc));

if isempty(minus_min_direc)
    minus_min_direc = pi;
end
if isempty(plus_min_direc)
    plus_min_direc = pi;
end
for itable = 1:table_length
    candidate_state_direc = atan2(lookuptable(itable,2),lookuptable(itable,1));

    if plus_min_direc<candidate_state_direc || -minus_min_direc>candidate_state_direc
        lookuptable(itable,11) = (lookuptable(itable,9)/max_speed_eval ...
            +lookuptable(itable,10)/max_direction_eval ...
            )/2;
    else
        lookuptable(itable,11) = lookuptable(itable,9)/max_speed_eval...
            +lookuptable(itable,10)/max_direction_eval;
    end
end

lookuptable = sortrows(lookuptable,11,'descend');
%lookuptable(1,:)
%disp(lookuptable);
candidate_point = zeros(1,4);
candidate_point_2 = zeros(1,4);
%is_collision_2 = 0;
%is_collision = 0;
for itable = 1:table_length
    
    candidate_point(1:3) = x(1:3)'+lookuptable(itable,1:3);
    candidate_point(4) = 1.4;
    
    if abs(candidate_point(2)) > 2
        continue;
    end
    is_collision = collision_check_for_simulator_ver2(candidate_point,existence_prohability_distribution_all);%変更点
    if is_collision == 1
        continue;
    end
    r_candidate_point = lookuptable(itable,1:3);
    path_array_pre = optimize_trajectory(r_candidate_point,x(5),lookuptable(itable,6:8),[0,0,0,x(4),x(5)]);
    if isnan(path_array_pre)
        continue;
    end
    path_length = size(path_array_pre,1);
    for ipath = 2:path_length-1
        %is_collision_2 = 0;
        candidate_point_2(1:3) = x(1:3)'+path_array_pre(ipath,1:3);
        candidate_point_2(4) = (ipath-1)/10;
        
        if abs(candidate_point_2(2)) >= 2
            is_collision_2 = 1;
            break;
        end
        is_collision_2 = collision_check_for_simulator_ver2(candidate_point_2,existence_prohability_distribution_all);%変更点
        if is_collision_2 == 1
            break;
        end
    end
    if is_collision_2 == 0
        path_array = path_array_pre;
        %disp(itable)
        %disp(table_length)
        break;
    end
end

end
%{
function last_position_and_p = search_nearest_one_from_lookuptable(x,y,yaw,lookuptable,counter)
    min_d = 100;
    min_id = 0;
    table_length = counter;
    
    for i = 1:table_length
        dx = x - lookuptable(i,1);
        dy = y - lookuptable(i,2);
        dyaw = yaw - lookuptable(i,3);
        %dv = v - lookuptable(i,4);
        %dk0 = k0 - lookuptable(i,5);
        d = norm([dx,dy,dyaw]);
        if d <= min_d
            min_id = i;
            min_d = d;
        end
    end
    
    last_position_and_p = lookuptable(min_id,:);
end
%}
function [path_array,p,max_steer_angle] = optimize_trajectory(target, k0, p,x)
h = [0.01,0.01,0.01]';%ニュートン法で最適値の探索をする際の微小偏差，加速度，中間舵角，終端舵角の順
for i = 1:30%この値は調整がいるかも
    path_array = motion_model_base_trajectory_genarator_nb(p(1),p(2),p(3),k0,x);
    %{
    steer_angle_array = path_array_pre(:,5);
    abs_angle_array = abs(steer_angle_array);
    if max(abs_angle_array)<deg2rad(35)
        path_array = path_array_pre;
    end
    %}
    if isnan(path_array)
        
    else
        xc = path_array(:,1);
        yc = path_array(:,2);
        yawc = path_array(:,3);
        steer_angle = path_array(:,5);
        abs_steer_angle = abs(steer_angle);
        max_steer_angle = max(abs_steer_angle);
        number = length(xc);
        last_state = [xc(number),yc(number),yawc(number)]';
        if size(target) ~= size(last_state)
            last_state = last_state';
        end
        dc = target - last_state;
        cost = norm(target-last_state);
        cost_th = 0.1;
    
        if cost <= cost_th 
               break;
        end

        J = parameter_optimization(target,p,h,k0,x);
        is_zero_vector = zeros(3,1);%特定変数だけの平衡点で処理が止まらないように
        if J(1,1) == 0 && J(2,1) == 0 && J(3,1) == 0
            is_zero_vector(1) = 1;
        end
        if J(1,2) == 0 && J(2,2) == 0 && J(3,2) == 0
            is_zero_vector(2) = 1;
        end
        if J(1,3) == 0 && J(2,3) == 0 && J(3,3) == 0
            is_zero_vector(3) = 1;
        end
        dc_t = dc';
        for izero = 1:3
           if is_zero_vector(4-izero)==1
               J(:,4-izero) = [];
               J(4-izero,:) = [];
               dc_t(4-izero) = [];
           end
        end
        if isempty(J)
            path_array = NaN;
            break;
        end
        dp_pre = -J\dc_t;
        dp = zeros(3,1);
        not_zero_array = find(is_zero_vector == 0);
        num = length(not_zero_array);
        for ip = 1:num
            dp(not_zero_array(ip)) = dp_pre(ip);
        end
        alpha = selection_learning_param(dp,p,k0,target,x);
        p_pre = p;
        p = p + alpha * dp;
        p(1) = min([0.5,max([p(1),-0.5])]);
        max_km = 0.7 * deg2rad(50) + k0;
        min_km = -0.7 * deg2rad(50) + k0;
        p(2) = min([max_km,max([p(2),min_km])]);
        max_kf = 0.7 * deg2rad(50) + p(2);
        min_kf = -0.7 * deg2rad(50) + p(2);
        p(3) = min([max_kf,max([p(3),min_kf])]);
        if p_pre(1) == p(1) && p_pre(2) == p(2) && p_pre(3) == p(3)
            path_array = NaN;
            break
        end
    end
end

end
  
function last_state = generate_last_state(p1,p2,p3,k0,x)%p1:加速度，p2:中間操舵角，p3:終端操舵角
    path_array = motion_model_base_trajectory_genarator_nb(p1,p2,p3,k0,x);%加速度，中間地点の操舵角，目標操舵角，初期操舵角
    path_length = size(path_array,1);
    last_state = path_array(path_length,1:3);
end

function error_matrix = parameter_optimization(target,p,h,k0,x)

    last_state_p = generate_last_state(...
        p(1) + h(1), p(2), p(3), k0,x);
    dp = target -last_state_p;
    last_state_n = generate_last_state(...
        p(1) - h(1), p(2), p(3), k0,x);
    dn = target -last_state_n;
    d1 = ((dp - dn) / (2.0 * h(1)))';

    last_state_p = generate_last_state(...
        p(1), p(2) + h(2), p(3), k0,x);
    dp = target -last_state_p;
    last_state_n = generate_last_state(...
        p(1), p(2) - h(2), p(3), k0,x);
    dn = target -last_state_n;
    d2 = ((dp - dn) / (2.0 * h(2)))';

    last_state_p = generate_last_state(...
        p(1), p(2), p(3) + h(3), k0,x);
    dp = target -last_state_p;%positive
    last_state_n = generate_last_state(...
        p(1), p(2), p(3) - h(3), k0,x);
    dn = target -last_state_n;%negative
    d3 = ((dp - dn) / (2.0 * h(3)))';

    error_matrix = [d1,d2,d3];
    
end

function learning_parameter = selection_learning_param(dp,p,k0,target,x)
    mincost = inf;
    learning_parameter = 0.2;

    a = linspace(0.2,2,10);

    for i = 1:10
        tp = p + a(i) * dp;
        last_state = generate_last_state(...
            tp(1), tp(2), tp(3), k0,x);
        dc = target-last_state;
        cost = norm(dc);
    

        if cost <= mincost
            if a ~= 0.0
                learning_parameter = a(i);
                mincost = cost;
            end
        end
    end
end
function angle = pi_2_pi(rad)
    angle = rad;
    if rad > 2*pi
        a = floor(rad/(2*pi));
        angle = angle - 2.0 * pi *a;
    end
    if rad < -2*pi
        a = ceil(abs(rad)/(2*pi));
        angle = angle + 2.0 * pi * a;
    end
    
    if angle > pi
        angle = angle - 2*pi;
    end
    
    if angle < -pi
        angle = angle + 2*pi;
    end
end