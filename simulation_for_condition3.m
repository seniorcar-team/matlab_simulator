function emergency_array = simulation_for_condition3(Pedestrian,file_num)

%　SFMの実行に際して，現在速度を維持するような処理を追加した,位置の記録は0.1秒おき
% 　歩行周期を導入
% シミュレーション空間の削減，20*4m
% ver5との変更点，SFMver6を使う，初期歩行者配置がセニアカーの近くになりすぎないように．停止してしまったときの処理

close all;
%clear all;
%str_iteration = num2str(iteration_num);

 
disp('Program start!!')

set(0, 'defaultAxesFontSize', 11);
set(0, 'defaultAxesFontName', 'Times New Roman');
set(0, 'defaultAxesXGrid', 'off');
set(0, 'defaultAxesYGrid', 'off');
set(0, 'defaultAxesGridAlpha', 1);
set(0, 'defaultAxesBox', 'off');
set(0, 'defaultAxesGridLineStyle', ':');
set(0, 'defaultTextFontSize', 11);
set(0, 'defaultTextFontName', 'Times New Roman');
set(0, 'defaultLineLineWidth', 1.5); 
set(0, 'defaultAxesLineWidth', 1.5);
set(0, 'defaultFigurePosition',[100 100 500 500]);
% % set(0, 'defaultLineLineWidth', 3); 
% % set(0, 'defaultAxesLineWidth', 3);
% fig=figure()
% % set(gcf,'PaperPositionMode','auto')
% set(fig,'PaperPositionMode','auto')
% set(fig,'Position',[10 10 1670 1040]);
global road_width; road_width = 10.0;%幅4mの道
global road_length; road_length = 20;%長さ20mの道
%% 車両パラメータの設定 ここは基本変更なし
global dt; dt=1/10; %計算に使用するdt
global dt_sim; dt_sim=1/10; %アニメ－ションの刻み時間[s]
global pedestrian_width; pedestrian_width = 400*10^-3;%歩行者幅[m]
global vehicle_width; vehicle_width=650*10^-3;%車両幅[m]
global vehicle_length; vehicle_length = 1100*10^-3;%車長[m]
global vehicle_front_length; vehicle_front_length = 900*10^-3;%後輪軸か先端までの距離[m]
global vehicle_rear_length; vehicle_rear_length = vehicle_length - vehicle_front_length;%後輪軸から後端までの距離[m]
global desired_speed; desired_speed = 4.0/3.6; %目標速度[m/s]
% ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),delta(rad)] delta=操舵角
x=[0 0 0 3.6/3.6 0]'; 
goal=[road_length,0,0];%ゴールの位置 [x(m),y(m)]
Sf =[0 0];%[v,yaw]
ind =1;%index 
max_acceleration = 1.5;
angle_increment = 0.25/180*pi;
%ロボットの力学モデル
%[最高速度[m/s],最高操舵角[rad],最高加減速度[m/ss],最高角速度[rad/s],
% 速度解像度[m/s],操舵角解像度[rad]]
Kinematic=[6.0/3.6,toRadian(35.0),1.5,toRadian(50.0),0.05,toRadian(2)];


%　周辺状況の設定
%% 壁の位置 ここら辺は適当に調整
% global wall_left; wall_left = 2.5; %壁の座標（左手）[m]
% global wall_right; wall_right = -1.5; %壁の座標（右手）[m]
global wall_left; wall_left = road_width/2; %壁の座標（左手）[m]
global wall_right; wall_right = -road_width/2; %壁の座標（右手）[m]
%壁の配置 [x(m) y(m)]


% plot用の壁の座標
x_wall_over = [0 road_length road_length 0];
y_wall_over = [wall_right wall_right wall_right-1 wall_right-1];
x_wall_under = [0 road_length road_length 0];
y_wall_under = [wall_left wall_left wall_left+1 wall_left+1];

%% 歩行者の設定 （読み込むからいらない）
Pedestrian_ver3 = Pedestrian;
ped_num = length(Pedestrian_ver3);
%%
%シミュレーション結果

result.x=zeros(1000,7); %車両状態Xの格納+フラッグと衝突判定
result.ped = zeros(length(Pedestrian_ver3),1000,2);
result.ped_speed = zeros(length(Pedestrian_ver3),1000);
pedestrian_first_state_matrix = zeros(length(Pedestrian_ver3),5);
for num = 1:ped_num
    pedestrian_first_state_matrix(num,1) = Pedestrian_ver3(num).position(1);
    pedestrian_first_state_matrix(num,2) = Pedestrian_ver3(num).position(2);
    pedestrian_first_state_matrix(num,3) = Pedestrian_ver3(num).speed;
    pedestrian_first_state_matrix(num,4) = Pedestrian_ver3(num).direction;
    pedestrian_first_state_matrix(num,5) = Pedestrian_ver3(num).target_position(1);
    pedestrian_first_state_matrix(num,6) = Pedestrian_ver3(num).target_position(2);
end
%% 動画作成の準備

   % 動画保存用フォルダを作成（現在のディレクトリに追加）
    fold_name = ['動画'];                           % フォルダ名の入力　←動画を保存するフォルダ名を入力する
%     mkdir(fold_name);                             % フォルダの作成
    
   % ディレクトリの変更
%     cd(fold_name);                                  % 作成したフォルダに移動
    
   % 動画ファイルの作成（MP-4)
    dd = datestr(now,'mmdd_HHMM');
    file = ('condition3');
    file_num_str =num2str(file_num);
    file_name = [file, '_',file_num_str, '_',dd];
    file_mp4 = VideoWriter(file_name, 'MPEG-4'); % 動画ファイル作成　←ファイル名を入力
    file_mp4.FrameRate = 10;                        % [ 1秒あたりのフレーム数を変更したい場合(既定値30) ]
    open(file_mp4);
    
    %結果配列の保存
stop_counter=0;%停止した時間が一定を超えたら処理をとめる
%% mainloop
existence_prohability_distrubution = zeros(road_length*10+1,road_width*10+1,41,ped_num);
%existence_prohability_distrubution_all = zeros(201,41,41);
predict_obserbable_pedestrian_position = zeros(ped_num,46);
predict_obserbable_pedestrian_position(:,3:46) = NaN;
loop_finish_num = 0;
time_pre = 0;
path_generation_flag = 1;
stop_counter_flag =1;
stop_counter_flag_ =1;
stop_time_2 = 0;
check_time = 0;
cruising_suppression_flag = 1;
path_start_time =0;
following_supression = 0;%ゴールが近づいたときだけ，１になる
goal_follow_suppression_flag =0;
%↓この範囲をほかのシミュレーションプログラムにコピペ
emergency_array = [0,0];
collision_counter = 0;
stop_number_counter = 0;
is_collision_pre=1;%ぶつからないとき1だから
is_collision_now=1;%上と同じ，一応置いてあるがおそらく不要
seniorcar_speed_pre = x(4);
%↑この範囲をほかのシミュレーションプログラムにコピペ
for i=1:1000
    t = (i-1)*dt_sim;

       
    Pedestrian_ver3 = Pedestrian_next_step_ver2(Pedestrian_ver3,0.5);%歩行周期一回分の時間後の予測される位置の計算，estimated_positionのみ更新
    


    %シミュレーション結果の保存
    result.x(i,1:5)=x';
    for num = 1:length(Pedestrian_ver3)
        result.ped(num,i,1) = Pedestrian_ver3(num).tmp_position(1);
        result.ped(num,i,2) = Pedestrian_ver3(num).tmp_position(2);
        result.ped_speed(num,i) = norm(Pedestrian_ver3(num).velocity);
    end
    
    %セニアカーの観測
    occupancy_matrix_for_seniorcar = caluculate_occupanted_area(x,Pedestrian_ver3);
    %代表位置のθ，ｒ，観測されて時刻が記録されている
    predict_obserbable_pedestrian_position = predict_position_for_simulator_ver3(occupancy_matrix_for_seniorcar,predict_obserbable_pedestrian_position,x,Pedestrian_ver3);   
    %パス形成のタイミングがいい感じになるように調整
    A_ = occupancy_matrix_for_seniorcar(2,:);
    B_ = A_(A_~=0);%距離が格納されているもののうち０でないもの
    [range_min,range_min_ind] = min(B_);%非ゼロの距離のうち最小のものと，非ゼロの配列の中での順番
    C_ind = find(A_~=0);%非ゼロものが元の配列で何番目のインデックスかが記録された配列
    %[range_min,range_min_ind] = min(occupancy_matrix_for_seniorcar(2,:));
    
    D_ = find(~isnan(predict_obserbable_pedestrian_position(:,3)));
    cruising_flag =0;
    following_flag =0;
    goal_follow_flag = 0;
    stop_flag =0;
    danger_flag = 0;
    path_generation_flag_3 =  0;
    if path_generation_flag ==1
        r_goal = goal(1:2)-x(1:2)';
        goal_distance = norm(r_goal);
        time_to_goal = goal_distance/x(4);
        goal_angle = atan2(r_goal(2),r_goal(1));
        observable_ped_num = length(D_);
        
        for iped = 1:observable_ped_num
            if predict_obserbable_pedestrian_position(D_(iped),4) < 5
                
                
                
                r_velocity = [x(4)*cos(x(3)),x(4)*sin(x(3))]-Pedestrian_ver3(D_(iped)).velocity;
                r_position = Pedestrian_ver3(D_(iped)).tmp_position-x(1:2)';
                if 1.0>sqrt(norm(r_position)^2-(dot(r_position,r_velocity)/norm(r_velocity))^2)
                    distance = norm(r_position)-1.1;
                    speed = dot(r_position,r_velocity)/norm(r_position);
                    ttc = distance/speed;
                    if ttc<0 || ttc>time_to_goal
                        continue
                    elseif ttc<time_to_goal && ttc<1.9
                        path_generation_flag_3 = 1;
                        disp('?')
                        break;
                    end

                end
            end
        end
    end
    if isempty(C_ind)
        ped_r_position_y = 10;
        range_min =10;
    else
        ped_r_position_y = range_min*sin(-pi/2+angle_increment*(C_ind(range_min_ind)-1));
    end
    if range_min<0.3 && ped_r_position_y < 0.4
        path_generation_flag_2 =0;
        u = [max([0,x(4)-max_acceleration*dt]),x(5)];
        %disp('hard decceleration');
        cruising_flag =0;
        following_flag =0;
        stop_flag =0;
        danger_flag = 1;
        cruising_suppression_flag = 1;
    elseif isnan(range_min)
        disp('$')
        path_generation_flag_2 =0;
        u = [max([0,x(4)-max_acceleration*dt]),x(5)];
        %disp('hard decceleration');
        cruising_flag =0;
        following_flag =0;
        stop_flag =0;
        danger_flag = 1;
        cruising_suppression_flag = 1;
    else
        path_generation_flag_2 = 1;
    end
    if path_generation_flag_3 == 1 && path_generation_flag_2 == 1 
        %各歩行者存在確率分布の計算 ここの関数を作る
        
        existence_prohability_distrubution = state_transition_model_ver11(x,predict_obserbable_pedestrian_position...
            ,existence_prohability_distrubution,Pedestrian_ver3,i,time_pre);
        time_pre = i;%処理が終わった後に確率分布作成時の時刻を記録
        %全歩行者の確率分布の合成
        existence_prohability_distrubution_all = zeros(road_length*10+1,road_width*10+1,41); 
        for k = 1:41
            for j = 1:ped_num
            existence_prohability_distrubution_all(:,:,k)...
                = existence_prohability_distrubution_all(:,:,k)+existence_prohability_distrubution(:,:,k,j);%matlabの配列でーたの取り扱いの問題で添え字とデータの連携を変更した
            end
        end

    %存在確率分布のマップに対してのパス形成
    
        
        
        path_array = state_lattice_motion_planner_for_c1_c3(x,existence_prohability_distrubution_all(:,:,:),goal,predict_obserbable_pedestrian_position,i,Pedestrian_ver3);%現在時刻から，2秒先までの確率分布を用いる
    %何秒分のパスが引けるか
        
        path_start_time = i;
        path_generation_flag = 0;%一度パスを作成したら，パス上の走行が終わるまで
        disp('!');
        cruising_suppression_flag = 0;
        
    elseif path_generation_flag_3 == 0 && path_generation_flag_2 == 1 && goal_follow_suppression_flag == 0%goal_follow中に壁にぶつからないように調整
        if x(2)+0.9*sin(x(3))+0.325*sin(x(3)+pi/2) >= road_width/2-1 && x(3) >=0
%            u = [min([1.1,x(4)+0.5*0.1/2]),max([-(35*pi/180.0),x(5)-(50*pi/180.0)*0.1])];
            u = [min([1.1,x(4)+0.5*0.1/2]),max([-Kinematic(2),x(5)-Kinematic(4)*0.1])];
        elseif x(2)+0.9*sin(x(3))+0.325*sin(x(3)-pi/2) <= -road_width/2+1 && x(3) <= 0
%            u = [min([1.1,x(4)+0.5*0.1/2]),min([(35*pi/180.0),x(5)+(50*pi/180.0)*0.1])];
            u = [min([1.1,x(4)+0.5*0.1/2]),min([Kinematic(2),x(5)+Kinematic(4)*0.1])];
        else
            disp('&')
            r_goal = goal(1:2)-x(1:2)';
            goal_distance = norm(r_goal);
            goal_angle = atan2(r_goal(2),r_goal(1));
            if goal_angle-x(3) ~= 0
                turning_radius = norm(r_goal)/2/(sin(goal_angle-x(3)));
            else
                turning_radius = 100000;
            end
            %if x(2)+turning_radius*sin(x(3)+pi/2)
            u = [min([1.1,x(4)+0.5*0.1/2]),atan(0.9/turning_radius)];
        end
        goal_follow_flag = 1;
        cruising_flag =0;
        following_flag =0;
        stop_flag =0;
        danger_flag = 0;
    end

    %走行戦略の判断 ここを書く
    if goal_follow_flag == 0 && danger_flag == 0
         if isnan(path_array)
            u_pre=following_ver12(x,goal,Pedestrian_ver3,predict_obserbable_pedestrian_position);%速度とステアリングアングルが返ってくる,車両制約範囲になるように，ポテンシャルの値に重みをかける．
            if u_pre(3) == 1 &&following_supression == 0 %following出来るとき
                u = u_pre(1:2);%uには速度と操舵角が入るように
                %disp('following1');
                disp('b')
                cruising_flag =0;
                following_flag =1;
                stop_flag =0;
                danger_flag = 0;
                cruising_suppression_flag =1;
                goal_follow_suppression_flag = 1;
                if  norm(x(1:2)'-goal(1:2)) < 5.0
                    path_generation_flag =1;
                    
                    goal_follow_suppression_flag = 0;
                    following_supression = 1;
                end


            else
                
                if x(3)>pi/6
                    u = [max([0,x(4)-max_acceleration*dt/4]),x(5)-(50*pi/180.0)*dt];%ブレーキかけて減速させる．この段階では危険があるわけではないので軽い減速
                elseif x(3)<-pi/6
                    u = [max([0,x(4)-max_acceleration*dt/4]),x(5)+(50*pi/180.0)*dt];
                else
                    u = [max([0,x(4)-max_acceleration*dt/4]),x(5)];
                end
                disp('c')
                %disp('mild decceleration2');
                cruising_flag =0;
                following_flag =0;
                stop_flag =1;
                danger_flag = 0;
                if stop_counter_flag ==1
                    stop_time = i;
                    stop_counter_flag =0;
                end
                if stop_counter_flag == 0
                    stop_time_difference = i - stop_time;
                    cruising_suppression_flag = 1;
                    goal_follow_suppression_flag = 1;
                    if stop_time_difference > 5
                        path_generation_flag = 1;
                        goal_follow_suppression_flag = 0;
                        stop_counter_flag =1;
                    end

                end
                
            end
         else
            if cruising_suppression_flag ==0



                if (i - path_start_time)<4
                    %disp('cruising2');
                    u = path_array(i - path_start_time+2,4:5);%一つ目のものは現在位置が格納されているため．
                    goal_follow_suppression_flag =1;
                elseif (i - path_start_time)==4%0.5秒目の入力をするタイミング
                    %disp('cruising2');
                    u = path_array(i - path_start_time+2,4:5);
                    path_generation_flag = 1;
                    goal_follow_suppression_flag =0;
                end
                disp('d-1')
                cruising_flag =1;
                following_flag =0;
                stop_flag =0;   
                danger_flag = 0;
                u_pre=following_ver12(x,goal,Pedestrian_ver3,predict_obserbable_pedestrian_position);%速度とステアリングアングルが返ってくる,車両制約範囲になるように，ポテンシャルの値に重みをかける．
                if u_pre(3) == 1 && following_supression == 0
                    u_follow = u_pre(1:2);%uには速度と操舵角が入るように
                    acceleration_cruising = u(1)^2*sin(u(2))/0.9;
                    acceleration_following = u_follow(1)^2*sin(u_follow(2))/0.9;
                    if abs(acceleration_cruising) > abs(acceleration_following) && abs(acceleration_cruising)>0.4
                        u = u_follow;
                        disp('d-2')
                        %disp('following2');
                        cruising_flag =0;
                        following_flag =1;
                        stop_flag =0;
                        danger_flag = 0;
                        cruising_suppression_flag =1;
                        goal_follow_suppression_flag = 1;
                        if  norm(x(1:2)'-goal(1:2)) < 5.0
                            path_generation_flag =1;
                            goal_follow_suppression_flag = 0;
                            following_supression = 1;
                        end
                    end
                end
            else
                u_pre=following_ver12(x,goal,Pedestrian_ver3,predict_obserbable_pedestrian_position);%速度とステアリングアングルが返ってくる,車両制約範囲になるように，ポテンシャルの値に重みをかける．
                if u_pre(3) == 1 && following_supression == 0
                    u = u_pre(1:2);%uには速度と操舵角が入るように
                    %disp('following1');
                    disp('e')
                    cruising_flag =0;
                    following_flag =1;
                    stop_flag =0;
                    danger_flag = 0;
                    cruising_suppression_flag =1;
                    goal_follow_suppression_flag = 1;
                    if  norm(x(1:2)'-goal(1:2)) < 5.0
                        path_generation_flag =1;
                        goal_follow_suppression_flag = 0;
                        following_supression = 1;
                    end
                else
                        if x(3)>pi/6
                            u = [max([0,x(4)-max_acceleration*dt/4]),x(5)-(50*pi/180.0)*dt];%ブレーキかけて減速させる．この段階では危険があるわけではないので軽い減速
                        elseif x(3)<-pi/6
                            u = [max([0,x(4)-max_acceleration*dt/4]),x(5)+(50*pi/180.0)*dt];
                        else
                            u = [max([0,x(4)-max_acceleration*dt/4]),x(5)];
                        end
                        disp('e')
                        cruising_flag =0;
                        following_flag =0;
                        stop_flag =1;
                        danger_flag = 0;
                        if stop_counter_flag ==1
                            stop_time = i;
                            stop_counter_flag =0;
                        end
                        if stop_counter_flag == 0
                            stop_time_difference = i - stop_time;
                            cruising_suppression_flag = 1;
                            goal_follow_suppression_flag = 1;
                            if stop_time_difference > 5
                                path_generation_flag = 1;
                                goal_follow_suppression_flag = 0;
                                stop_counter_flag =1;
                            end

                        end
                end

            end
         end
    end
    if range_min<0.3 && ped_r_position_y < 0.4
        path_generation_flag = 1;
    end
    % セニアカーの状態更新
    x =  f_sim(x,u);
    
    x  = real(x);
    if x(4) == 0
        zero_speed_counter = zero_speed_counter + 1;
        if zero_speed_counter > 6
            path_generation_flag = 1;
        end
    end
    if x(4) ~=  0
        zero_speed_counter = 0;
    end
    
    for num = 1:length(Pedestrian_ver3)%歩行者位置の更新
        Pedestrian_ver3(num).velocity = SocialForce_pedestrian_ver9(x,Pedestrian_ver3,num);
        Pedestrian_ver3(num) = plusStep(Pedestrian_ver3(num),dt_sim);
        Pedestrian_ver3(num).tmp_position = [Pedestrian_ver3(num).tmp_position(1) max([-road_width/2 min([road_width/2 Pedestrian_ver3(num).tmp_position(2)])])];%壁から出ないように    
    end

    %停止して進まなくなった時の処理
    
    if x(4) == 0
        if stop_counter ==0
            stop_counter = i;
        end
        if i-stop_counter>1200%速度が0になってからこの処理に入り，この処理の前でstop_counterに代入を行うので，stop_counterが0のままこの処理に入ってすぐプログラムが止まることはないはず
            disp('failed!!');
            result.x(i+1:1000,:)=[];
            result.ped(:,i+1:1000,:) = [];
            result.ped_speed(:,i+1:1000) = [];
            loop_finish_num = i;
            is_collision_now = Judgement_collision(x,Pedestrian_ver3);
            result.x(i,6) = 1*cruising_flag+3*following_flag+4*stop_flag+5*danger_flag;%ちょっとややこしくなってる
            result.x(i,7) = is_collision_now;
            break;
        end
    end
    
    
    %ゴール判定

    if norm(x(1:2)'-goal(1:2))<1.5
        disp('Arrive Goal!!');
        result.x(i+1:1000,:)=[];
        result.ped(:,i+1:1000,:) = [];
        result.ped_speed(:,i+1:1000) = [];
        loop_finish_num = i;
        is_collision_now = Judgement_collision(x,Pedestrian_ver3);
        result.x(i,6) = 1*cruising_flag+3*following_flag+4*stop_flag+5*danger_flag;%ちょっとややこしくなってる
        result.x(i,7) = is_collision_now;
        break;%到達したら終了
    end

    % 車両の4隅の座標の算出　x_Sfは後輪の中間地点の座標
    x_car_Sf(1) = x(1)+vehicle_front_length*cos(x(3))-vehicle_width/2*sin(x(3));
    y_car_Sf(1) = x(2)+vehicle_front_length*sin(x(3))+vehicle_width/2*cos(x(3));
    x_car_Sf(2) = x(1)-vehicle_rear_length*cos(x(3))-vehicle_width/2*sin(x(3));
    y_car_Sf(2) = x(2)-vehicle_rear_length*sin(x(3))+vehicle_width/2*cos(x(3));
    x_car_Sf(3) = x(1)-vehicle_rear_length*cos(x(3))+vehicle_width/2*sin(x(3));
    y_car_Sf(3) = x(2)-vehicle_rear_length*sin(x(3))-vehicle_width/2*cos(x(3));
    x_car_Sf(4) = x(1)+vehicle_front_length*cos(x(3))+vehicle_width/2*sin(x(3));
    y_car_Sf(4) = x(2)+vehicle_front_length*sin(x(3))-vehicle_width/2*cos(x(3));
    abs_y = abs(y_car_Sf);
    if max(abs_y) > road_width/2
        disp("incident occured!")
        is_collision_now = Judgement_collision(x,Pedestrian_ver3);
        result.x(i,6) = 1*cruising_flag+3*following_flag+4*stop_flag+5*danger_flag;%ちょっとややこしくなってる
        result.x(i,7) = is_collision_now;
        break
    end
    %====Animation====

    %hold on;

    %歩行者の描画
    hold off;
    
    
    plot(result.x(:,1),result.x(:,2),'-r'); %車両軌跡
    plot(goal(1),goal(2),'*r');hold on;
%     plot(pedself(1),pedself(2),'*b');
    if cruising_flag ==1
        fill(x_car_Sf,y_car_Sf,'r'); %車両のボデーの描画
    end
    if following_flag ==1
        fill(x_car_Sf,y_car_Sf,'b'); %車両のボデーの描画
    end
    if goal_follow_flag ==1
        fill(x_car_Sf,y_car_Sf,'y'); %車両のボデーの描画
    end
    if stop_flag ==1
        fill(x_car_Sf,y_car_Sf,'k'); %車両のボデーの描画
    end
    if danger_flag ==1
        fill(x_car_Sf,y_car_Sf,'g'); %車両のボデーの描画

    end
    
    
    %↓ここから
    is_collision_now = Judgement_collision(x,Pedestrian_ver3);
    result.x(i,6) = 1*cruising_flag+3*following_flag+4*stop_flag+5*danger_flag;%ちょっとややこしくなってる
    result.x(i,7) = is_collision_now;
    if is_collision_now - is_collision_pre == -1
        collision_counter = collision_counter + 1;
    end
    is_collision_pre = is_collision_now;
    %speedがゼロの時speedの逆数はinfなので
    if isinf(1/seniorcar_speed_pre)+isinf(1/x(4)) == -1
        stop_number_counter = stop_number_counter + 1;
    end
    seniorcar_speed_pre = x(4);
    %↑ここまでコピペする
    
    %plot(result.x(:,1),result.x(:,2),'-r'); %車両軌跡
    fill(x_wall_over,y_wall_over,[0.3 0.3 0.3]); %壁の描画
    fill(x_wall_under,y_wall_under,[0.3 0.3 0.3]); %壁の描画
    quiver(x(1),x(2),x(4)*cos(x(3)),x(4)*sin(x(3)),2,'k','Linewidth',2,'MaxHeadSize',2);%矢印プロット
    if path_generation_flag == 1
        text(0,5,'generate path','Color','red','FontSize',14);
    else
        text(0,5,'.','Color','blue','FontSize',14);
    end
    for num = 1:length(Pedestrian_ver3)
        cx = 1; cy = 1; % 中心        
        
        if ~isnan(predict_obserbable_pedestrian_position(num,3))
%            viscircles(Pedestrian_ver3(num).tmp_position,pedestrian_width,'Color','b');hold on;
            viscircles(Pedestrian_ver3(num).tmp_position,pedestrian_width/2,'EdgeColor','b');hold on;
        else
%            viscircles(Pedestrian_ver3(num).tmp_position,pedestrian_width,'Color','y');hold on;
            viscircles(Pedestrian_ver3(num).tmp_position,pedestrian_width/2,'EdgeColor','y');hold on;
        end
        
    end
    fill(x_wall_over,y_wall_over,[0.3 0.3 0.3]); %壁の描画
    fill(x_wall_under,y_wall_under,[0.3 0.3 0.3]); %壁の描画

    grid on;
    axis equal;
    xlim([0 20]);
    ylim([-3 3]);

    
    xlabel('X [m]');
    ylabel('Y [m]');
    drawnow;
    
%     動画の保存  
    frame = getframe(1);              % 現在表示しているfigureをキャプチャ
    writeVideo(file_mp4, frame);      % 動画ファイルにキャプチャを保存
    hold off;
    
end

B = [];
D = [];
F = [];
H = [];
J = [];
L = [];
N = [];
file_name_first_position_ped = ['pedestrian_first_state_condition3_',file_num_str,'_',dd];
Z = pedestrian_first_state_matrix;
save(file_name_first_position_ped,'Z','-ascii')

close(file_mp4); 

k = [];
l = [];
i_num = size(result.x,1);
for i = 1:i_num
    k = result.x(i,:);
    l =[l;k];
end
if ischar(file_num)
    file_num_str = file_num;
else
    file_num_str = num2str(file_num);
end
filename = ['seniorcar_state_condition3_',file_num_str,'_',dd];
save(filename,'l','-ascii');
emergency_array = [collision_counter,stop_number_counter];

% cd ..
%toc;
