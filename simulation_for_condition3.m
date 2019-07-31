function emergency_array = simulation_for_condition3(Pedestrian,file_num)

%�@SFM�̎��s�ɍۂ��āC���ݑ��x���ێ�����悤�ȏ�����ǉ�����,�ʒu�̋L�^��0.1�b����
% �@���s�����𓱓�
% �V�~�����[�V������Ԃ̍팸�C20*4m
% ver5�Ƃ̕ύX�_�CSFMver6���g���C�������s�Ҕz�u���Z�j�A�J�[�̋߂��ɂȂ肷���Ȃ��悤�ɁD��~���Ă��܂����Ƃ��̏���

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
global road_width; road_width = 10.0;%��4m�̓�
global road_length; road_length = 20;%����20m�̓�
%% �ԗ��p�����[�^�̐ݒ� �����͊�{�ύX�Ȃ�
global dt; dt=1/10; %�v�Z�Ɏg�p����dt
global dt_sim; dt_sim=1/10; %�A�j���|�V�����̍��ݎ���[s]
global pedestrian_width; pedestrian_width = 400*10^-3;%���s�ҕ�[m]
global vehicle_width; vehicle_width=650*10^-3;%�ԗ���[m]
global vehicle_length; vehicle_length = 1100*10^-3;%�Ԓ�[m]
global vehicle_front_length; vehicle_front_length = 900*10^-3;%��֎�����[�܂ł̋���[m]
global vehicle_rear_length; vehicle_rear_length = vehicle_length - vehicle_front_length;%��֎������[�܂ł̋���[m]
global desired_speed; desired_speed = 4.0/3.6; %�ڕW���x[m/s]
% ���{�b�g�̏������[x(m),y(m),yaw(Rad),v(m/s),delta(rad)] delta=���Ǌp
x=[0 0 0 3.6/3.6 0]'; 
goal=[road_length,0,0];%�S�[���̈ʒu [x(m),y(m)]
Sf =[0 0];%[v,yaw]
ind =1;%index 
max_acceleration = 1.5;
angle_increment = 0.25/180*pi;
%���{�b�g�̗͊w���f��
%[�ō����x[m/s],�ō����Ǌp[rad],�ō��������x[m/ss],�ō��p���x[rad/s],
% ���x�𑜓x[m/s],���Ǌp�𑜓x[rad]]
Kinematic=[6.0/3.6,toRadian(35.0),1.5,toRadian(50.0),0.05,toRadian(2)];


%�@���ӏ󋵂̐ݒ�
%% �ǂ̈ʒu ������ӂ͓K���ɒ���
% global wall_left; wall_left = 2.5; %�ǂ̍��W�i����j[m]
% global wall_right; wall_right = -1.5; %�ǂ̍��W�i�E��j[m]
global wall_left; wall_left = road_width/2; %�ǂ̍��W�i����j[m]
global wall_right; wall_right = -road_width/2; %�ǂ̍��W�i�E��j[m]
%�ǂ̔z�u [x(m) y(m)]


% plot�p�̕ǂ̍��W
x_wall_over = [0 road_length road_length 0];
y_wall_over = [wall_right wall_right wall_right-1 wall_right-1];
x_wall_under = [0 road_length road_length 0];
y_wall_under = [wall_left wall_left wall_left+1 wall_left+1];

%% ���s�҂̐ݒ� �i�ǂݍ��ނ��炢��Ȃ��j
Pedestrian_ver3 = Pedestrian;
ped_num = length(Pedestrian_ver3);
%%
%�V�~�����[�V��������

result.x=zeros(1000,7); %�ԗ����X�̊i�[+�t���b�O�ƏՓ˔���
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
%% ����쐬�̏���

   % ����ۑ��p�t�H���_���쐬�i���݂̃f�B���N�g���ɒǉ��j
    fold_name = ['����'];                           % �t�H���_���̓��́@�������ۑ�����t�H���_������͂���
%     mkdir(fold_name);                             % �t�H���_�̍쐬
    
   % �f�B���N�g���̕ύX
%     cd(fold_name);                                  % �쐬�����t�H���_�Ɉړ�
    
   % ����t�@�C���̍쐬�iMP-4)
    dd = datestr(now,'mmdd_HHMM');
    file = ('condition3');
    file_num_str =num2str(file_num);
    file_name = [file, '_',file_num_str, '_',dd];
    file_mp4 = VideoWriter(file_name, 'MPEG-4'); % ����t�@�C���쐬�@���t�@�C���������
    file_mp4.FrameRate = 10;                        % [ 1�b������̃t���[������ύX�������ꍇ(����l30) ]
    open(file_mp4);
    
    %���ʔz��̕ۑ�
stop_counter=0;%��~�������Ԃ����𒴂����珈�����Ƃ߂�
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
following_supression = 0;%�S�[�����߂Â����Ƃ������C�P�ɂȂ�
goal_follow_suppression_flag =0;
%�����͈̔͂��ق��̃V�~�����[�V�����v���O�����ɃR�s�y
emergency_array = [0,0];
collision_counter = 0;
stop_number_counter = 0;
is_collision_pre=1;%�Ԃ���Ȃ��Ƃ�1������
is_collision_now=1;%��Ɠ����C�ꉞ�u���Ă��邪�����炭�s�v
seniorcar_speed_pre = x(4);
%�����͈̔͂��ق��̃V�~�����[�V�����v���O�����ɃR�s�y
for i=1:1000
    t = (i-1)*dt_sim;

       
    Pedestrian_ver3 = Pedestrian_next_step_ver2(Pedestrian_ver3,0.5);%���s������񕪂̎��Ԍ�̗\�������ʒu�̌v�Z�Cestimated_position�̂ݍX�V
    


    %�V�~�����[�V�������ʂ̕ۑ�
    result.x(i,1:5)=x';
    for num = 1:length(Pedestrian_ver3)
        result.ped(num,i,1) = Pedestrian_ver3(num).tmp_position(1);
        result.ped(num,i,2) = Pedestrian_ver3(num).tmp_position(2);
        result.ped_speed(num,i) = norm(Pedestrian_ver3(num).velocity);
    end
    
    %�Z�j�A�J�[�̊ϑ�
    occupancy_matrix_for_seniorcar = caluculate_occupanted_area(x,Pedestrian_ver3);
    %��\�ʒu�̃ƁC���C�ϑ�����Ď������L�^����Ă���
    predict_obserbable_pedestrian_position = predict_position_for_simulator_ver3(occupancy_matrix_for_seniorcar,predict_obserbable_pedestrian_position,x,Pedestrian_ver3);   
    %�p�X�`���̃^�C�~���O�����������ɂȂ�悤�ɒ���
    A_ = occupancy_matrix_for_seniorcar(2,:);
    B_ = A_(A_~=0);%�������i�[����Ă�����̂̂����O�łȂ�����
    [range_min,range_min_ind] = min(B_);%��[���̋����̂����ŏ��̂��̂ƁC��[���̔z��̒��ł̏���
    C_ind = find(A_~=0);%��[�����̂����̔z��ŉ��Ԗڂ̃C���f�b�N�X�����L�^���ꂽ�z��
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
        %�e���s�ґ��݊m�����z�̌v�Z �����̊֐������
        
        existence_prohability_distrubution = state_transition_model_ver11(x,predict_obserbable_pedestrian_position...
            ,existence_prohability_distrubution,Pedestrian_ver3,i,time_pre);
        time_pre = i;%�������I�������Ɋm�����z�쐬���̎������L�^
        %�S���s�҂̊m�����z�̍���
        existence_prohability_distrubution_all = zeros(road_length*10+1,road_width*10+1,41); 
        for k = 1:41
            for j = 1:ped_num
            existence_prohability_distrubution_all(:,:,k)...
                = existence_prohability_distrubution_all(:,:,k)+existence_prohability_distrubution(:,:,k,j);%matlab�̔z��Ł[���̎�舵���̖��œY�����ƃf�[�^�̘A�g��ύX����
            end
        end

    %���݊m�����z�̃}�b�v�ɑ΂��Ẵp�X�`��
    
        
        
        path_array = state_lattice_motion_planner_for_c1_c3(x,existence_prohability_distrubution_all(:,:,:),goal,predict_obserbable_pedestrian_position,i,Pedestrian_ver3);%���ݎ�������C2�b��܂ł̊m�����z��p����
    %���b���̃p�X�������邩
        
        path_start_time = i;
        path_generation_flag = 0;%��x�p�X���쐬������C�p�X��̑��s���I���܂�
        disp('!');
        cruising_suppression_flag = 0;
        
    elseif path_generation_flag_3 == 0 && path_generation_flag_2 == 1 && goal_follow_suppression_flag == 0%goal_follow���ɕǂɂԂ���Ȃ��悤�ɒ���
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

    %���s�헪�̔��f ����������
    if goal_follow_flag == 0 && danger_flag == 0
         if isnan(path_array)
            u_pre=following_ver12(x,goal,Pedestrian_ver3,predict_obserbable_pedestrian_position);%���x�ƃX�e�A�����O�A���O�����Ԃ��Ă���,�ԗ�����͈͂ɂȂ�悤�ɁC�|�e���V�����̒l�ɏd�݂�������D
            if u_pre(3) == 1 &&following_supression == 0 %following�o����Ƃ�
                u = u_pre(1:2);%u�ɂ͑��x�Ƒ��Ǌp������悤��
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
                    u = [max([0,x(4)-max_acceleration*dt/4]),x(5)-(50*pi/180.0)*dt];%�u���[�L�����Č���������D���̒i�K�ł͊댯������킯�ł͂Ȃ��̂Ōy������
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
                    u = path_array(i - path_start_time+2,4:5);%��ڂ̂��̂͌��݈ʒu���i�[����Ă��邽�߁D
                    goal_follow_suppression_flag =1;
                elseif (i - path_start_time)==4%0.5�b�ڂ̓��͂�����^�C�~���O
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
                u_pre=following_ver12(x,goal,Pedestrian_ver3,predict_obserbable_pedestrian_position);%���x�ƃX�e�A�����O�A���O�����Ԃ��Ă���,�ԗ�����͈͂ɂȂ�悤�ɁC�|�e���V�����̒l�ɏd�݂�������D
                if u_pre(3) == 1 && following_supression == 0
                    u_follow = u_pre(1:2);%u�ɂ͑��x�Ƒ��Ǌp������悤��
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
                u_pre=following_ver12(x,goal,Pedestrian_ver3,predict_obserbable_pedestrian_position);%���x�ƃX�e�A�����O�A���O�����Ԃ��Ă���,�ԗ�����͈͂ɂȂ�悤�ɁC�|�e���V�����̒l�ɏd�݂�������D
                if u_pre(3) == 1 && following_supression == 0
                    u = u_pre(1:2);%u�ɂ͑��x�Ƒ��Ǌp������悤��
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
                            u = [max([0,x(4)-max_acceleration*dt/4]),x(5)-(50*pi/180.0)*dt];%�u���[�L�����Č���������D���̒i�K�ł͊댯������킯�ł͂Ȃ��̂Ōy������
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
    % �Z�j�A�J�[�̏�ԍX�V
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
    
    for num = 1:length(Pedestrian_ver3)%���s�҈ʒu�̍X�V
        Pedestrian_ver3(num).velocity = SocialForce_pedestrian_ver9(x,Pedestrian_ver3,num);
        Pedestrian_ver3(num) = plusStep(Pedestrian_ver3(num),dt_sim);
        Pedestrian_ver3(num).tmp_position = [Pedestrian_ver3(num).tmp_position(1) max([-road_width/2 min([road_width/2 Pedestrian_ver3(num).tmp_position(2)])])];%�ǂ���o�Ȃ��悤��    
    end

    %��~���Đi�܂Ȃ��Ȃ������̏���
    
    if x(4) == 0
        if stop_counter ==0
            stop_counter = i;
        end
        if i-stop_counter>1200%���x��0�ɂȂ��Ă��炱�̏����ɓ���C���̏����̑O��stop_counter�ɑ�����s���̂ŁCstop_counter��0�̂܂܂��̏����ɓ����Ă����v���O�������~�܂邱�Ƃ͂Ȃ��͂�
            disp('failed!!');
            result.x(i+1:1000,:)=[];
            result.ped(:,i+1:1000,:) = [];
            result.ped_speed(:,i+1:1000) = [];
            loop_finish_num = i;
            is_collision_now = Judgement_collision(x,Pedestrian_ver3);
            result.x(i,6) = 1*cruising_flag+3*following_flag+4*stop_flag+5*danger_flag;%������Ƃ�₱�����Ȃ��Ă�
            result.x(i,7) = is_collision_now;
            break;
        end
    end
    
    
    %�S�[������

    if norm(x(1:2)'-goal(1:2))<1.5
        disp('Arrive Goal!!');
        result.x(i+1:1000,:)=[];
        result.ped(:,i+1:1000,:) = [];
        result.ped_speed(:,i+1:1000) = [];
        loop_finish_num = i;
        is_collision_now = Judgement_collision(x,Pedestrian_ver3);
        result.x(i,6) = 1*cruising_flag+3*following_flag+4*stop_flag+5*danger_flag;%������Ƃ�₱�����Ȃ��Ă�
        result.x(i,7) = is_collision_now;
        break;%���B������I��
    end

    % �ԗ���4���̍��W�̎Z�o�@x_Sf�͌�ւ̒��Ԓn�_�̍��W
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
        result.x(i,6) = 1*cruising_flag+3*following_flag+4*stop_flag+5*danger_flag;%������Ƃ�₱�����Ȃ��Ă�
        result.x(i,7) = is_collision_now;
        break
    end
    %====Animation====

    %hold on;

    %���s�҂̕`��
    hold off;
    
    
    plot(result.x(:,1),result.x(:,2),'-r'); %�ԗ��O��
    plot(goal(1),goal(2),'*r');hold on;
%     plot(pedself(1),pedself(2),'*b');
    if cruising_flag ==1
        fill(x_car_Sf,y_car_Sf,'r'); %�ԗ��̃{�f�[�̕`��
    end
    if following_flag ==1
        fill(x_car_Sf,y_car_Sf,'b'); %�ԗ��̃{�f�[�̕`��
    end
    if goal_follow_flag ==1
        fill(x_car_Sf,y_car_Sf,'y'); %�ԗ��̃{�f�[�̕`��
    end
    if stop_flag ==1
        fill(x_car_Sf,y_car_Sf,'k'); %�ԗ��̃{�f�[�̕`��
    end
    if danger_flag ==1
        fill(x_car_Sf,y_car_Sf,'g'); %�ԗ��̃{�f�[�̕`��

    end
    
    
    %����������
    is_collision_now = Judgement_collision(x,Pedestrian_ver3);
    result.x(i,6) = 1*cruising_flag+3*following_flag+4*stop_flag+5*danger_flag;%������Ƃ�₱�����Ȃ��Ă�
    result.x(i,7) = is_collision_now;
    if is_collision_now - is_collision_pre == -1
        collision_counter = collision_counter + 1;
    end
    is_collision_pre = is_collision_now;
    %speed���[���̎�speed�̋t����inf�Ȃ̂�
    if isinf(1/seniorcar_speed_pre)+isinf(1/x(4)) == -1
        stop_number_counter = stop_number_counter + 1;
    end
    seniorcar_speed_pre = x(4);
    %�������܂ŃR�s�y����
    
    %plot(result.x(:,1),result.x(:,2),'-r'); %�ԗ��O��
    fill(x_wall_over,y_wall_over,[0.3 0.3 0.3]); %�ǂ̕`��
    fill(x_wall_under,y_wall_under,[0.3 0.3 0.3]); %�ǂ̕`��
    quiver(x(1),x(2),x(4)*cos(x(3)),x(4)*sin(x(3)),2,'k','Linewidth',2,'MaxHeadSize',2);%���v���b�g
    if path_generation_flag == 1
        text(0,5,'generate path','Color','red','FontSize',14);
    else
        text(0,5,'.','Color','blue','FontSize',14);
    end
    for num = 1:length(Pedestrian_ver3)
        cx = 1; cy = 1; % ���S        
        
        if ~isnan(predict_obserbable_pedestrian_position(num,3))
%            viscircles(Pedestrian_ver3(num).tmp_position,pedestrian_width,'Color','b');hold on;
            viscircles(Pedestrian_ver3(num).tmp_position,pedestrian_width/2,'EdgeColor','b');hold on;
        else
%            viscircles(Pedestrian_ver3(num).tmp_position,pedestrian_width,'Color','y');hold on;
            viscircles(Pedestrian_ver3(num).tmp_position,pedestrian_width/2,'EdgeColor','y');hold on;
        end
        
    end
    fill(x_wall_over,y_wall_over,[0.3 0.3 0.3]); %�ǂ̕`��
    fill(x_wall_under,y_wall_under,[0.3 0.3 0.3]); %�ǂ̕`��

    grid on;
    axis equal;
    xlim([0 20]);
    ylim([-3 3]);

    
    xlabel('X [m]');
    ylabel('Y [m]');
    drawnow;
    
%     ����̕ۑ�  
    frame = getframe(1);              % ���ݕ\�����Ă���figure���L���v�`��
    writeVideo(file_mp4, frame);      % ����t�@�C���ɃL���v�`����ۑ�
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
