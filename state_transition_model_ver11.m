function existence_prohability_distribution_next_step = state_transition_model_ver11(x,predict_position_matrix,existence_prohability_distribution,Pedestrian,time,time_pre)
%まだ未完,以前のバージョン分の動作確認完了，後は観測された位置の履歴をもとに歩行者速度を出す部分を完成させる，2/27.1:05
%ver6との変更点はexistence_prohability_distributionのサイズを小さくしたものへの対応．時間ごとにデータを移行する
%時間のインデックスが１で現在時刻になるように調整，観測誤差の分はマージンで吸収するように調整
%引数の変更があった
%右辺について引数はセニアカーの位置，観測に基づく観測された歩行者の代表位置の距離と角度，簡単のために歩行者情報を用いる
%遷移モデルには現在の速さを保つという特性と，進行方向が異なるものを回避するという特性を単純化したものを反映させる
%簡単に作るためにシミュレーション上のデータを用いている，今後は修正する
%direction使わなかった
%angle_increment = 0.25/180*pi;
%ave_range_error = -0.11;
%std_range_error = 0.2;
%ave_angle_error = -0.0066;
%std_angle_error = 0.046;
dt = 0.1;
%existence_prohability_distribution(k,x,y,t)は観測された歩行者のうちk番目の歩行者kのx，ｙ，ｔにおける存在確率
%predict_position_matrixは行方向に歩行者番号で各行は歩行者番号，ゴールの方向，現在のθ，ｒ，現在のx，y，0.1秒前のx，y．．．，2秒前のx，y
%NaNは観測出来ていないとき
ped_num = size(existence_prohability_distribution,4);
x_counter = size(existence_prohability_distribution,1);
y_counter = size(existence_prohability_distribution,2);
time_counter = size(existence_prohability_distribution,3);


%predict_position_matrixは観測された歩行者について，歩行者番号，位置ｘ，ｙ，進行方向の2次元の値
C_ = find(~isnan(predict_position_matrix(:,5)));%現在時刻に観測された歩行者のインデックス.ここは旧バージョンと一緒で問題ない
ped_num2 = length(C_);%現在時刻に観測された歩行者数
existence_prohability_distribution_next_step = ones(x_counter,y_counter,time_counter,ped_num)./(x_counter*y_counter);%一様確率で分布させる
%size(existence_prohability_distribution_next_step)
%size(existence_prohability_distribution)
time_difference=time-time_pre;
if time_difference <= 40 && time_pre ~=0
    next_step_time_max_ind = 41-time_difference;
    last_step_time_min_ind = 1+time_difference;
    %last_step_time_max_ind = last_step_time_min_ind+(next_step_time_max_ind-1);
    existence_prohability_distribution_next_step(:,:,1:next_step_time_max_ind,:)=existence_prohability_distribution(:,:,last_step_time_min_ind:41,:);%以前の確率分布で使えるところは使う
end

% % for test by Yoshitake
% existence_prohability_distribution_next_step_new = existence_prohability_distribution_next_step;

for k = 1:ped_num%kはこれから存在確率分布を求める歩行者を示す，
    predict_position_array = predict_position_matrix(k,5:46);
    nearest_obseved_time_ind =find(~isnan(predict_position_array), 1 );
    if isempty(nearest_obseved_time_ind)%2秒以上観測無し
        continue
    end
    obseved_t = ceil(nearest_obseved_time_ind/2);
    time_difference = obseved_t-1;
    if time_difference >(time-time_pre)%以前の確率分布計算以降新たな観測が得られていない
        continue
    end
%    r_ped_to_ped = zeros(ped_num2,4);%各歩行者座標における極座標のtheta，ｒ，速度の順,kが変わるたびに初期化
%% 現在観測が得られた歩行者のみの処理
%{
    if predict_position_matrix(k,5) == time
        for ik = 1:ped_num2
        %そもそもpredict_position_matrixは行方向に歩行者番号順に並んでいるが，
        %要素の一つ目が歩行者番号になっている．
        %C_は現在時刻に観測された歩行者のインデックスが格納されている
        %現在時刻において観測された歩行者間の位置関係を記録
            if C_(ik)==k
                continue
            end
        
            r_ped_to_ped(ik,2)=norm(predict_position_matrix(C_(ik),2)-predict_position_matrix(k,2)...
                ,predict_position_matrix(C_(ik),3)-predict_position_matrix(k,3));
            r_ped_to_ped(ik,1)=atan2(predict_position_matrix(C_(ik),3)-predict_position_matrix(k,3),...
            predict_position_matrix(C_(ik),2)-predict_position_matrix(k,2));%グローバル座標系での角度
            r_ped_to_ped(ik,3)=Pedestrian(predict_position_matrix(C_(ik),1)).velocity(1);
            r_ped_to_ped(ik,4)=Pedestrian(predict_position_matrix(C_(ik),1)).velocity(2);
        end

    end
%}
%% 0.5秒前から現在時刻の間で観測が得られた歩行者のみ　↑の処理で観測が得られた時刻と現在時刻との差が2秒以上のものはcontinue文で抜けている
    %predict_position_matrix(k,1)
    %k
    %rとθの1σ区間の4点のx，y座標を出す←　観測誤差はマージンで吸収するからパーティクルの部分を消す
    %{
    for m = 1:2
        for n = 1:2
            [now_ped_r_position_x,now_ped_r_position_y] = ...
                pol2cart(predict_position_matrix_for_simulator(k,1)+x(3)-(std_angle_error)+2*(m-1)*std_angle_error,...
                predict_position_matrix_for_simulator(k,2)-(std_range_error)+2*(m-1)*std_range_error);
            now_pedestrian_position(2*(m-1)+n,:) = [x(1)+0.9*cos(x(3))+now_ped_r_position_x...
                ,x(2)+0.9*sin(x(3))+now_ped_r_position_y];
        end
    end
    
    for o = 1:4
        now_ped_position_ind(o,:) = [ 1+round(now_pedestrian_position(o,1),1)*10,...
            21+round(now_pedestrian_position(o,2),1)*10];%existence_prohability_distribution_matrixのインデックスと合わせた形式
    end
    
    x_max = max(now_ped_position_ind(:,1));
    x_min = min(now_ped_position_ind(:,1));
    x_num = x_max-x_min +1;
    y_max = max(now_ped_position_ind(:,2));
    y_min = min(now_ped_position_ind(:,2));
    y_num = y_max-y_min +1;
    Np = x_num*y_num;
    
    particle = zeros(Np,3);%位置ｘ，ｙと重みを格納
    for ix = 1:x_num
        for iy=1:y_num
            x_position = 0.1*(x_min-1+ix-1);
            y_position = -2+0.1*(y_min-1+iy-1);
            distance_to_particle = norm(x_position-(x(1)+0.9*cos(x(3))),y_position- (x(2)+0.9*sin(x(3))));%観測点はセニアカーの前方のため
            theta_to_particle =atan2(y_position- (x(2)+0.9*sin(x(3))),x_position-(x(1)+0.9*cos(x(3))))-x(3);
            observation_prohability_range = Gauss...
                (distance_to_particle,predict_position_matrix_for_simulator(k,2),std_range_error);%観測が得られた際の，ある位置の歩行者の存在確率(ｒ方向について)
            observation_prohability_angle = Gauss...
                (theta_to_particle,predict_position_matrix_for_simulator(k,1),std_angle_error);
            observation_prohability = observation_prohability_range*observation_prohability_angle;
            particle(y_num*(ix-1)+iy,:) = [x_position,y_position,observation_prohability];
        end
    end
    %パーティクルの重みの計算までは問題なかった
    %}
    %% 観測が得られた時間との対応付けを行う
    for l = 1:time_counter%何秒先まで見るか，(かける0.1秒後まで)
        %ver6で歩行者の進行方向ごとに条件付けする．
        
        min_i = max([1,floor(10*predict_position_matrix(k,nearest_obseved_time_ind+4))-ceil(2*(time_difference+l-1))]);%xおよびｙは0.1mでインデックスが１変わる
        max_i = min([201,ceil(10*predict_position_matrix(k,nearest_obseved_time_ind+4))+ceil(2*(time_difference+l-1))]);
        if predict_position_matrix(k,2) == 1
            min_i = min([x_counter,max([1,floor(10*predict_position_matrix(k,nearest_obseved_time_ind+4))])]);
        end
        if predict_position_matrix(k,2) == 0
            max_i = min([x_counter,max([1,floor(10*predict_position_matrix(k,nearest_obseved_time_ind+4))])]);
        end
        tMod = tic;
        %===========================
        % for test by Yoshitake
        %===========================
        i_num = max_i - min_i + 1;
        r_position_new = zeros(i_num, 41, 2);
        for i = min_i : max_i
            for j = 1 : 41
                
                r_position_new(i-min_i+1,j,:) = [0.1*(i-1),-2+0.1*(j-1)] - predict_position_matrix(k,nearest_obseved_time_ind+4:nearest_obseved_time_ind+5);
                
            end
        end
        %観測代表点の履歴を使って歩行者の速度表現を行う
        direction_new = atan2(r_position_new(:,:,2),r_position_new(:,:,1));
        direction_new = round(direction_new,5);
        velocity_new = r_position_new ./ ((time_difference+l-1)*dt);
        velocity_norm_new = sqrt(velocity_new(:,:,1).^2 + velocity_new(:,:,2).^2);
        velocity_norm_new = round(velocity_norm_new,5);
        speed_prohability_new = 1/sqrt(2*pi*(0.5*(4.0/3.6)^2))*exp((-1/2)*(velocity_norm_new-(4.0/3.6)).^2/((0.5*(4.0/3.6))^2));
        angle_prohability_new = (sqrt((velocity_norm_new.^(-1).*0.3 .*2 .*pi).^2)).^(-1).*exp((-1/2)*abs(pi_2_pi((direction_new-atan2(Pedestrian(k).velocity(2),Pedestrian(k).velocity(1))))).^2./((0.3./velocity_norm_new).^2));

        for i = min_i : max_i
            for j = 1 : 41
                
                existence_prohability_distribution_next_step(i,j,l,k) = ...
                    existence_prohability_distribution_next_step(i,j,l,k) ...
                    + speed_prohability_new(i-min_i+1,j) * angle_prohability_new(i-min_i+1,j);
%                     existence_prohability_distribution_next_step_new(i,j,time_counter-20+l,k) = ...
%                         existence_prohability_distribution_next_step_new(i,j,time_counter-20+l,k) ...
%                         + particle(ip,3) * speed_prohability_new(i-min_i+1,j,ip) * angle_prohability_new(i-min_i+1,j,ip);

            end
        end
        
%         dist_new = existence_prohability_distribution_next_step_new;
%         dist_norm_new = sum(sum(existence_prohability_distribution_next_step_new(:,:,time_counter-20+l,k)));
        
%         existence_prohability_distribution_next_step_new(:,:,time_counter-20+l,k) = ...
%             existence_prohability_distribution_next_step_new(:,:,time_counter-20+l,k)/sum(sum(existence_prohability_distribution_next_step_new(:,:,time_counter-20+l,k)));
        existence_prohability_distribution_next_step(:,:,l,k) = ...
            existence_prohability_distribution_next_step(:,:,l,k)/sum(sum(existence_prohability_distribution_next_step(:,:,l,k)));
        %===========================
 
%         velocity_norm_org = zeros(i_num, 41, Np);
%         direction_org = zeros(i_num, 41, Np);
%         speed_prohability_org = zeros(i_num, 41, Np);
%         angle_prohability_org = zeros(i_num, 41, Np);
 
        tModElapsed = toc(tMod);
%         fprintf('t=%d,k=%2d,l=%2d,Mod=%5.3f\n', time, k, l, tModElapsed);

        % 元コードのコメントアウト by Yoshitake
%         tOrg = tic;
%        for i = min_i:max_i%観測された点から生成したパーティクルのxの範囲±2m分（1秒間の設定の時，歩行者速度がせいぜい1.7[m/s]なことを考えれば，サンプリング位置はこれで十分）
%             for j = 1:41%ｙ方向はそもそも±2mだから何もしない
%             
%                 sampling_position = [0.1*(i-1),-2+0.1*(j-1)];
%                 for ip = 1:Np
%                     r_position = sampling_position-particle(ip,1:2);
%                     velocity = r_position/((time-predict_position_matrix(k,5)+l)*dt);%サンプリングした点を何秒後の点とするかで速度を変える
%                     direction = atan2(r_position(2),r_position(1));
%                     %正直現在観測が得られていない歩行者に対して速度を使うのはどうかと思うが，調整の時間がなかったため，まだ調整していない．
%                     %セニアカーの速度が流速を反映しているとして，x（４）を平均として扱うのが一つの案
% %                    speed_prohability = normal_prohability_distribution_function...
% %                        (norm(velocity),norm(Pedestrian(k).velocity),...
% %                        0.5*norm(Pedestrian(k).velocity));
%                     speed_prohability = normal_prohability_distribution_function...
%                         (round(norm(velocity),5),norm(Pedestrian(k).velocity),...
%                         0.5*norm(Pedestrian(k).velocity));
% %                    angle_prohability_1 = normal_prohability_distribution_function...
% %                        (direction...
% %                        ,atan2(Pedestrian(k).velocity(2),Pedestrian(k).velocity(1))...
% %                        ,0.3/norm(Pedestrian(k).velocity));
%                     angle_prohability_1 = normal_prohability_distribution_function...
%                         (round(direction,5) ...
%                         ,atan2(Pedestrian(k).velocity(2),Pedestrian(k).velocity(1))...
%                         ,0.3/norm(Pedestrian(k).velocity));
%                     %0.5秒間の周方向の位置連れが標準偏差0.15mだから，角度の標準偏差を0.3/speedとした．
%                     angle_prohability_2 = 1;
%                     
%                     velocity_norm_org(i-min_i+1,j,ip) = round(norm(velocity),5);
%                     direction_org(i-min_i+1,j,ip) = round(direction,5);
%                     speed_prohability_org(i-min_i+1,j,ip) = speed_prohability;
%                     angle_prohability_org(i-min_i+1,j,ip) = angle_prohability_1;
%                     
%                     % Comment out by Yoshitake
% %                     %1は進行方向を保ちやすい性質から，2は障害物以外の方向に進む性質から
% %                     if predict_position_matrix(k,5) == time
% %                         [~,index]=min(abs(r_ped_to_ped(:,1)-direction)); 
% %                         r_v =  [velocity(1)-r_ped_to_ped(index,3),velocity(2)-r_ped_to_ped(index,4)];
% %                         r_p = [r_ped_to_ped(index,2)*cos(r_ped_to_ped(index,1)),r_ped_to_ped(index,2)*sin(r_ped_to_ped(index,1))];
% %                         if dot(r_p,r_v)/norm(r_v)>sqrt(norm(r_p)^2-0.4^2)%0.4は歩行者同士の衝突円半径，誤差を考えるなら0.4に何か加える必要がある
% %                             angle_prohability_2 = 0.5;%衝突可能性が高い方向に進むと予想する確率を下げるための重み，ここは調整が必要
% %                         else
% %                             angle_prohability_2 = 1;
% %                         end
% %                     end
% 
%                     existence_prohability_distribution_next_step(i,j,time_counter-20+l,k) = existence_prohability_distribution_next_step(i,j,time_counter-20+l,k)...
%                         +particle(ip,3)*speed_prohability*angle_prohability_1*angle_prohability_2;
%                 end
%                 %sampling_position確認のために使った
%             end
%         end
%         %正規化処理，各歩行者の各ステップごとの存在確率の和が１になるように調整
%         
%         dist_org = existence_prohability_distribution_next_step;
%         dist_norm_org = sum(sum(existence_prohability_distribution_next_step(:,:,time_counter-20+l,k)));
% 
%         existence_prohability_distribution_next_step(:,:,time_counter-20+l,k)...
%         =existence_prohability_distribution_next_step(:,:,time_counter-20+l,k)...
%             /sum(sum(existence_prohability_distribution_next_step(:,:,time_counter-20+l,k)));
%         tOrgElapsed = toc(tOrg);
        
%         fprintf('t=%d,k=%2d,l=%2d,Org=%5.3f,Mod=%5.3f,%d,%d,%d,%d,%d,%d,%d\n', ...
%             time, k, l, tOrgElapsed, tModElapsed, ...
%             sum(sum(sum(velocity_norm_new == velocity_norm_org))) == i_num*41*Np, ...
%             sum(sum(sum(speed_prohability_new == speed_prohability_org))) == i_num*41*Np, ...
%             sum(sum(sum(direction_new == direction_org))) == i_num*41*Np, ...
%             sum(sum(sum(angle_prohability_new == angle_prohability_org))) == i_num*41*Np, ...
%             sum(sum(sum(sum(dist_new == dist_org)))) == 201*41*41*20, ...
%             dist_norm_new == dist_norm_org, ...
%             sum(sum(sum(sum(round(existence_prohability_distribution_next_step,10) == round(existence_prohability_distribution_next_step_new,10))))) == 201*41*41*20 ...
%             );
    end
    
end  
end
function angle = pi_2_pi(rad)
    angle = rad;
    
    
    a = fix(rad./(2*pi));
    angle = angle - a .*2.0 .* pi;
    b = fix(angle./pi);
    angle = angle - b .*2.0 .* pi;
    
    angle(abs(angle)==pi) = 0;
end
%{
for ix = 1:201
    for iy = 1:41
        plot3(0.1*(ix-1),0.1*(iy-1),existence_prohability_distrubution_all(ix,iy,21),'.b');hold on;
    end
end
%}