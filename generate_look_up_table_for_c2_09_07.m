function [] = generate_look_up_table_for_c2_09_07()
velocity = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5,1.6];
%半周期?1.4秒で計算してる
yaw_resolution = 0.1;%ホイールベース的に車両先端位置が0.1m変わるのが大体角度0.1[rad]になるから
state_matrix_all = [];
for iv = 1:16
    %yaw = [];
    %theta = [];
    %state_matrix = [];
    x_for_optimization = [0,0,0,velocity(iv),0];
    abs_max_yaw = min([0.5*1.4/velocity(iv),pi/3]);
    yaw_number = ceil(abs_max_yaw/yaw_resolution);
    yaw_num = 2*yaw_number + 1;
    yaw = zeros(yaw_num,1);
    for iyaw = 1:yaw_num 
       yaw(iyaw) = -yaw_resolution*yaw_number + yaw_resolution*(iyaw-1);
    end
    theta_max = 0.4*velocity(iv)*1.4;%舵角一定の時の最大範囲
    theta_resolution = 1/14/velocity(iv);
    theta_number = round(theta_max / theta_resolution );
    theta_num = 2*theta_number +1;
    x = zeros(theta_num,1);
    y = zeros(theta_num,1);
    for itheta = 1:theta_num
        angle = -theta_resolution*theta_number+theta_resolution*(itheta-1);
        
        [x(itheta),y(itheta)] = pol2cart(...
            angle,velocity(iv)*1.4);
    end
    state_matrix = zeros(theta_num*yaw_num,3);
    for itheta = 1:theta_num
        for iyaw = 1:yaw_num
            state_matrix(iyaw+yaw_num*(itheta-1),:) = [x(itheta),y(itheta),yaw(iyaw)];
        end
    end
    state_matrix_all = [state_matrix_all;state_matrix];
end
%disp(state_matrix_all);
matrix_length = size(state_matrix_all,1);
    %% ここから実際にlookup_tableを作る
for iv = 1:3
    k0 = [-deg2rad(30),-deg2rad(25),-deg2rad(20),-deg2rad(15),-deg2rad(10),-deg2rad(5),0,...
        deg2rad(5),deg2rad(10),deg2rad(15),deg2rad(20),deg2rad(25),deg2rad(30)];

    %matrix_length = size(state_matrix_all,1);
    
    
    vel_str = num2str(velocity(17-iv)*10);
    for ik0 = 1:13
        counter = 0;
        lookuptable = zeros(matrix_length,8);
        k0_str = num2str(k0(ik0));
        x_for_optimization = [0,0,0,velocity(17-iv),k0(ik0)];
        for i_mat =1:matrix_length
            if counter>0
                nearest_p = search_nearest_one_from_lookuptable...
                    (state_matrix_all(i_mat,1),state_matrix_all(i_mat,2),state_matrix_all(i_mat,3),lookuptable,counter);
            else
                d = norm([state_matrix_all(i_mat,1),state_matrix_all(i_mat,2)]);
                initial_acceleration = 2*(d-1.4*velocity(14-iv))/1.96;
                angle_difference = atan2(state_matrix_all(i_mat,2),state_matrix_all(i_mat,1))-state_matrix_all(i_mat,3);
                steer = atan(2*0.9*sin(angle_difference)/d);
                steer_difference = steer - k0;
                nearest_p = [0,0,0,0,0,initial_acceleration...
                    ,min([max([-deg2rad(35),steer]),deg2rad(35)])...
                    ,min([max([-deg2rad(35),steer+steer_difference]),deg2rad(35)])];
            end
            target = [state_matrix_all(i_mat,1),state_matrix_all(i_mat,2),state_matrix_all(i_mat,3)];
            initial_p = [nearest_p(6),nearest_p(7),nearest_p(8)]';
            [path_array,p,max_steer_angle] = optimize_trajectory(target,k0(ik0),initial_p,x_for_optimization);
            path_length = size(path_array,1);
            if ~isnan(path_array) 
                if max_steer_angle <= deg2rad(35)
                    is_exist_same_state = 0;
                    last_state = [path_array(path_length,1),path_array(path_length,2),path_array(path_length,3)];
                    last_state = round(last_state,1);
                    for itable = 1:counter
                        is_exist_same_state = isequal(last_state,lookuptable(itable,1:3));
                        if is_exist_same_state == 1
                            break
                        end
                    end
                    if is_exist_same_state == 0
                        counter=counter+1;
                        lookuptable(counter,:) = [last_state,velocity(14-iv),k0(ik0),p'];
                    end
                end
            end
        end
        %disp(ik0);
        lookuptable(counter+1:matrix_length,:) = [];
        file_name = ['look_up_table_for_c2_c4_09_07_',vel_str,'_',k0_str];
        save(file_name,'lookuptable','-ascii');
    end
    
    disp(iv);

    
    
end
%save(filename,'lookuptable','-ascii');
end
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
    
function [path_array,p] = optimize_trajectory_generator_ver2(x,target_state)%target_stateは3行1列になるようにすること

target = target_state;

k0 = x(5);

init_p = [x(4)/10,x(5),x(5)]';%第一項の係数は加速度の時間変動値を見て調整,ターゲットをどの距離に置くか？，中間・終端舵角をどうするか

[path_array,p] = optimize_trajectory(target, k0, init_p,x);

end

function [path_array,p,max_steer_angle] = optimize_trajectory(target, k0, p,x)
h = [0.01,0.01,0.01]';%ニュートン法で最適値の探索をする際の微小偏差，加速度，中間舵角，終端舵角の順
for i = 1:30%この値は調整がいるかも
    path_array = motion_model_base_trajectory_genarator_for_c2_09_07(p(1),p(2),p(3),k0,x);
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
            break
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
        p(1) = min([0.25,max([p(1),-0.75])]);%実車実験から予想されたキャップ
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
    path_array = motion_model_base_trajectory_genarator_for_c2_09_07(p1,p2,p3,k0,x);%加速度，中間地点の操舵角，目標操舵角，初期操舵角
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
