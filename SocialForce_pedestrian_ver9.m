function Sf_ped = SocialForce_pedestrian_ver9(x,Pedestrian,num)
    %比較する車両の点を増やした．最も近い一点からの力を受けるように調整
    %現在速度をなるべく維持する処理を追加
    %セニアカーに対する回避が車両の後輪中心のみとなっているため変更する予定
    global wall_right;
    global wall_left;
    global dt_sim;
    global desired_speed;
    %% parameter
    sight_angle = 200; %[deg] 視野角
    max_velocity = 1.3 * desired_speed; %[m/s]

    %A = 0.04; %[m/s]
    %B = 3.22; %[m]
    %lamda = 0.06;%歩行者密度が高いときにはこっちのほうがいいかも？
    %B_seniorcar = 1.61;%歩行者のパーソナルスペースが
    
    A = 4.30;
    B = 1.07;
    lamda = 1.00;
    %B_seniorcar = 0.5;
    %U_alphaB = 10; 
    %R_U = 0.2;
    U_alphaB = A;
    R_U = B;
    tau_alpha = 0.5; %[s]%reference "The many roles of the relaxation time parameter in force based models of pedestrian dynamics"
    dr = 0.01; %[m]
    vehicle_front_size = 0.9;
    vehicle_rear_size = 0.2;
    vehicle_middle_size = 0.35;
    vehicle_width=0.65;
    pedestrian_radius = 0.2;
    % 衝突予測の条件
    global limit_time;
    global limit_dist;
    global limit_dist_obstacle;
    limit_time = 10; %[s]
    limit_dist = 3.6; %[m]
    limit_dist_obstacle = 1.2;%[m]
    %% initialize
    f_alphaB = [0 0];
    f_alphabeta =[0 0];
    f_alphabeta_all =[0 0];
    %% siniorcarの位置、速度
    x_seniorcar = x(1:2)';%位置を縦ベクトルに変更
    


    %% goal
    u=[x(4);x(5)];%入力値　vとdelta
    %% 歩行者
    v_pedestrian = Pedestrian(num).velocity;
    theta_pedestrian = atan2(v_pedestrian(2),v_pedestrian(1));
    %% goal  
    r_alphagoal = Pedestrian(num).target_position - Pedestrian(num).tmp_position;
    e_alphagoal = r_alphagoal / norm(r_alphagoal);%ゴール方向単位ベクトル
    f_goal = 1/tau_alpha*(e_alphagoal*desired_speed-v_pedestrian);
    
    wall = [wall_right wall_left];

    
    %% for boundary 
    
    
    for i = 1:length(wall)
        if Pedestrian(num).tmp_position(2) == wall(1)
            f_alphaB = [0 1];
            continue;
        end
        if Pedestrian(num).tmp_position(2) == wall(2)
            f_alphaB = [0 -1];
            continue;
        end
        r_alphaB = Pedestrian(num).tmp_position - [Pedestrian(num).tmp_position(1) wall(i)];
        e_alphaB = r_alphaB/norm(r_alphaB);
      %  r_alphaB_ = (norm(r_alphaB) - dr) * e_alphaB;%方向が同じで1㎝短いベクトル？
        F_alphaB = U_alphaB*exp(-norm(r_alphaB)/R_U);
      %  F_alphaB_ = U_alphaB*exp(-norm(r_alphaB_)/R_U);
            
        f_alphaB = f_alphaB+F_alphaB*e_alphaB;%12/17に変更,単位ベクトルの向きに注意
        %f_alphaB = f_alphaB-(F_alphaB-F_alphaB_)/dr*e_alphaB; なんでdrがついてたのかわからない
    end
    
    if Pedestrian(num).tmp_position(2) == wall(1)
        f_alphaB = [0 1];
    end
    if Pedestrian(num).tmp_position(2) == wall(2)
        f_alphaB = [0 -1];
    end

          %f_alphaB = 0.0;
    
 
    
    %% for pedestrian
    for number = 1:length(Pedestrian)
            % 相対ベクトル
       if num == number
           continue;
       end
       r_alphabeta = Pedestrian(num).tmp_position - Pedestrian(number).tmp_position;%相手の予測位置から自分
       
       if norm(r_alphabeta) > 5.0%閾値以上離れた存在は考慮しない
           continue
       end
       %ped_theta = rad2deg(atan2(-r_alphabeta(2),-r_alphabeta(1)));%ベクトルがx軸となす角
       %{
       sight_theta= abs(ped_theta - rad2deg(theta_pedestrian));%現進行方向ベクトルとなす角
       if sight_theta > 180
           sight_theta = 360 - sight_theta;
       end
       if sight_theta > sight_angle/2
            continue;
       else
       %}
          
           %歩行者から見た歩行者の方向（グローバル座標）
           theta_ped = atan2(-r_alphabeta(2),-r_alphabeta(1));
           theta_judge = theta_ped - theta_pedestrian;

            
           b_alphabeta = sqrt((norm(r_alphabeta)+norm(r_alphabeta-(Pedestrian(number).velocity - Pedestrian(num).velocity)*dt_sim))^2 ...
            -(norm(Pedestrian(number).velocity - Pedestrian(num).velocity)*dt_sim)^2)/2;
       
           y_alphabeta = (Pedestrian(number).velocity - Pedestrian(num).velocity) * dt_sim;
                
           w = lamda + (1 - lamda) * (1 + cos(theta_judge)) / 2;
           g = A*exp(-b_alphabeta/B)*(norm(r_alphabeta) + norm(r_alphabeta-y_alphabeta))...
           /(2*b_alphabeta)*(1/2)*(r_alphabeta/norm(r_alphabeta)+(r_alphabeta-y_alphabeta)/norm(r_alphabeta-y_alphabeta));%r_alphabetaはβからαの方向，
       
           f_alphabeta = w * g;
        


        

          f_alphabeta_all = f_alphabeta_all + f_alphabeta;
        %end
    end
    %% セニアカーとの相互作用,一点で考えると車両内部に入り込んでしまうため，代表6点についてSFMを適用する．
    
       x_seniorcar = x(1:2)';
       v_seniorcar = x(4)*[cos(x(3)) sin(x(3))];
       v_seniorcar = real(v_seniorcar);
       r_alphabeta = Pedestrian(num).tmp_position - x_seniorcar;%セニアカーから歩行者方向
       f_alphabeta_seniorcar =[0 0]; 
       if norm(r_alphabeta) > 5.0
           
       else
           ped_theta = rad2deg(atan2(-r_alphabeta(2),-r_alphabeta(1)));%ベクトルがx軸となす角
           %{
           sight_theta= abs(ped_theta - rad2deg(theta_pedestrian));%現進行方向ベクトルとなす角
           if  sight_theta > 180
            sight_theta = 360 - sight_theta;
           end
           if sight_theta > sight_angle/2
            f_alphabeta = [0 0];
           else
               %}
               %車両の四隅と中間の座標
               %{
               x_car(1,1) = x(1)+vehicle_front_size*cos(x(3))-vehicle_width/2*sin(x(3));
               x_car(1,2) = x(2)+vehicle_front_size*sin(x(3))+vehicle_width/2*cos(x(3));
               x_car(2,1) = x(1)-vehicle_rear_size*cos(x(3))-vehicle_width/2*sin(x(3));
               x_car(2,2) = x(2)-vehicle_rear_size*sin(x(3))+vehicle_width/2*cos(x(3));
               x_car(3,1) = x(1)-vehicle_rear_size*cos(x(3))+vehicle_width/2*sin(x(3));
               x_car(3,2) = x(2)-vehicle_rear_size*sin(x(3))-vehicle_width/2*cos(x(3));
               x_car(4,1) = x(1)+vehicle_front_size*cos(x(3))+vehicle_width/2*sin(x(3));
               x_car(4,2) = x(2)+vehicle_front_size*sin(x(3))-vehicle_width/2*cos(x(3));
               x_car(5,1) = x(1)+vehicle_middle_size*cos(x(3))-vehicle_width/2*sin(x(3));
               x_car(5,2) = x(2)+vehicle_middle_size*sin(x(3))+vehicle_width/2*cos(x(3));
               x_car(6,1) = x(1)+vehicle_middle_size*cos(x(3))+vehicle_width/2*sin(x(3));
               x_car(6,2) = x(2)+vehicle_middle_size*sin(x(3))-vehicle_width/2*cos(x(3));
               %}
               x_car(1,1) = x(1)+vehicle_front_size*cos(x(3));
               x_car(1,2) = x(2)+vehicle_front_size*sin(x(3));
               x_car(2,1) = x(1)+1/3*vehicle_front_size*cos(x(3));
               x_car(2,2) = x(2)+1/3*vehicle_front_size*sin(x(3));
               x_car(3,1) = x(1)+2/3*vehicle_front_size*cos(x(3));
               x_car(3,2) = x(2)+2/3*vehicle_front_size*sin(x(3));
               x_car(4,1) = x(1);
               x_car(4,2) = x(2);
               x_car(5,1) = x(1)-vehicle_rear_size*cos(x(3));
               x_car(5,2) = x(2)-vehicle_rear_size*sin(x(3));
               r =10;%次のループで距離が最も近いもののみからSFMを受けるように調整する
               for counter =1:5
                   r_pre = (-1)*(x_car(counter,:) - Pedestrian(num).tmp_position);%セニアカーから歩行者方向
                   if norm(r_pre) < norm(r)
                       r = r_pre;
                   else
                       continue;
                   end
                   theta_ped = atan2(-r(2),-r(1));%歩行者からセニアカーの方向のグローバル座標における方向
                   theta_judge = theta_ped - theta_pedestrian;%歩行者視点からの方向

                   y_alphabeta = (v_seniorcar-Pedestrian(num).velocity) * dt_sim;
                   b_alphabeta_fix = sqrt((norm(r)+norm(r-(v_seniorcar-Pedestrian(num).velocity)*dt_sim))^2 ...
                        -(norm(v_seniorcar-Pedestrian(num).velocity)*dt_sim)^2)/2;
                   w = lamda + (1 - lamda) * (1 + cos(theta_judge)) / 2;
                   g_fix = A*exp((0.2-b_alphabeta_fix)/B)*(norm(r) + norm(r-y_alphabeta))...
                        /(2*b_alphabeta_fix)*(1/2)*(r/norm(r)+(r-y_alphabeta)/norm(r-y_alphabeta));%seniorcarの大きさ分拡大
                  
                  f_alphabeta_seniorcar = w * g_fix;
                   
               end
           %シニアカーの大きさ考慮

           %歩行者から見たセニアカーの方向（グローバル座標）
       end
               f_alphabeta_all = f_alphabeta_all + f_alphabeta_seniorcar;
           %end
       
           
    
     
       
        
        
 
    %% 力の算出及び速度の計算
    f_social = f_goal + f_alphaB + f_alphabeta_all;

    
    Sf_ped = Pedestrian(num).velocity + f_social*dt_sim;
    %{
    if norm(Sf_ped) > 0.5*norm(Pedestrian(num).velocity) && norm(Sf_ped) <= norm(Pedestrian(num).velocity)
        Sf_ped = norm(Pedestrian(num).velocity)/norm(Sf_ped) * Sf_ped;
    end
    %}
    if norm(Sf_ped) > max_velocity
        Sf_ped = max_velocity/norm(Sf_ped) * Sf_ped;%maxvelocityの大きさのSf方向を持つベクトル
    end
    %a = norm(Sf_ped)
    
    
end