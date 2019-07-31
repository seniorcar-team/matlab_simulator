function Sf_ped = SocialForce_pedestrian_ver9(x,Pedestrian,num)
    %��r����ԗ��̓_�𑝂₵���D�ł��߂���_����̗͂��󂯂�悤�ɒ���
    %���ݑ��x���Ȃ�ׂ��ێ����鏈����ǉ�
    %�Z�j�A�J�[�ɑ΂��������ԗ��̌�֒��S�݂̂ƂȂ��Ă��邽�ߕύX����\��
    global wall_right;
    global wall_left;
    global dt_sim;
    global desired_speed;
    %% parameter
    sight_angle = 200; %[deg] ����p
    max_velocity = 1.3 * desired_speed; %[m/s]

    %A = 0.04; %[m/s]
    %B = 3.22; %[m]
    %lamda = 0.06;%���s�Җ��x�������Ƃ��ɂ͂������̂ق������������H
    %B_seniorcar = 1.61;%���s�҂̃p�[�\�i���X�y�[�X��
    
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
    % �Փ˗\���̏���
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
    %% siniorcar�̈ʒu�A���x
    x_seniorcar = x(1:2)';%�ʒu���c�x�N�g���ɕύX
    


    %% goal
    u=[x(4);x(5)];%���͒l�@v��delta
    %% ���s��
    v_pedestrian = Pedestrian(num).velocity;
    theta_pedestrian = atan2(v_pedestrian(2),v_pedestrian(1));
    %% goal  
    r_alphagoal = Pedestrian(num).target_position - Pedestrian(num).tmp_position;
    e_alphagoal = r_alphagoal / norm(r_alphagoal);%�S�[�������P�ʃx�N�g��
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
      %  r_alphaB_ = (norm(r_alphaB) - dr) * e_alphaB;%������������1�p�Z���x�N�g���H
        F_alphaB = U_alphaB*exp(-norm(r_alphaB)/R_U);
      %  F_alphaB_ = U_alphaB*exp(-norm(r_alphaB_)/R_U);
            
        f_alphaB = f_alphaB+F_alphaB*e_alphaB;%12/17�ɕύX,�P�ʃx�N�g���̌����ɒ���
        %f_alphaB = f_alphaB-(F_alphaB-F_alphaB_)/dr*e_alphaB; �Ȃ��dr�����Ă��̂��킩��Ȃ�
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
            % ���΃x�N�g��
       if num == number
           continue;
       end
       r_alphabeta = Pedestrian(num).tmp_position - Pedestrian(number).tmp_position;%����̗\���ʒu���玩��
       
       if norm(r_alphabeta) > 5.0%臒l�ȏ㗣�ꂽ���݂͍l�����Ȃ�
           continue
       end
       %ped_theta = rad2deg(atan2(-r_alphabeta(2),-r_alphabeta(1)));%�x�N�g����x���ƂȂ��p
       %{
       sight_theta= abs(ped_theta - rad2deg(theta_pedestrian));%���i�s�����x�N�g���ƂȂ��p
       if sight_theta > 180
           sight_theta = 360 - sight_theta;
       end
       if sight_theta > sight_angle/2
            continue;
       else
       %}
          
           %���s�҂��猩�����s�҂̕����i�O���[�o�����W�j
           theta_ped = atan2(-r_alphabeta(2),-r_alphabeta(1));
           theta_judge = theta_ped - theta_pedestrian;

            
           b_alphabeta = sqrt((norm(r_alphabeta)+norm(r_alphabeta-(Pedestrian(number).velocity - Pedestrian(num).velocity)*dt_sim))^2 ...
            -(norm(Pedestrian(number).velocity - Pedestrian(num).velocity)*dt_sim)^2)/2;
       
           y_alphabeta = (Pedestrian(number).velocity - Pedestrian(num).velocity) * dt_sim;
                
           w = lamda + (1 - lamda) * (1 + cos(theta_judge)) / 2;
           g = A*exp(-b_alphabeta/B)*(norm(r_alphabeta) + norm(r_alphabeta-y_alphabeta))...
           /(2*b_alphabeta)*(1/2)*(r_alphabeta/norm(r_alphabeta)+(r_alphabeta-y_alphabeta)/norm(r_alphabeta-y_alphabeta));%r_alphabeta�̓����烿�̕����C
       
           f_alphabeta = w * g;
        


        

          f_alphabeta_all = f_alphabeta_all + f_alphabeta;
        %end
    end
    %% �Z�j�A�J�[�Ƃ̑��ݍ�p,��_�ōl����Ǝԗ������ɓ��荞��ł��܂����߁C��\6�_�ɂ���SFM��K�p����D
    
       x_seniorcar = x(1:2)';
       v_seniorcar = x(4)*[cos(x(3)) sin(x(3))];
       v_seniorcar = real(v_seniorcar);
       r_alphabeta = Pedestrian(num).tmp_position - x_seniorcar;%�Z�j�A�J�[������s�ҕ���
       f_alphabeta_seniorcar =[0 0]; 
       if norm(r_alphabeta) > 5.0
           
       else
           ped_theta = rad2deg(atan2(-r_alphabeta(2),-r_alphabeta(1)));%�x�N�g����x���ƂȂ��p
           %{
           sight_theta= abs(ped_theta - rad2deg(theta_pedestrian));%���i�s�����x�N�g���ƂȂ��p
           if  sight_theta > 180
            sight_theta = 360 - sight_theta;
           end
           if sight_theta > sight_angle/2
            f_alphabeta = [0 0];
           else
               %}
               %�ԗ��̎l���ƒ��Ԃ̍��W
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
               r =10;%���̃��[�v�ŋ������ł��߂����݂̂̂���SFM���󂯂�悤�ɒ�������
               for counter =1:5
                   r_pre = (-1)*(x_car(counter,:) - Pedestrian(num).tmp_position);%�Z�j�A�J�[������s�ҕ���
                   if norm(r_pre) < norm(r)
                       r = r_pre;
                   else
                       continue;
                   end
                   theta_ped = atan2(-r(2),-r(1));%���s�҂���Z�j�A�J�[�̕����̃O���[�o�����W�ɂ��������
                   theta_judge = theta_ped - theta_pedestrian;%���s�Ҏ��_����̕���

                   y_alphabeta = (v_seniorcar-Pedestrian(num).velocity) * dt_sim;
                   b_alphabeta_fix = sqrt((norm(r)+norm(r-(v_seniorcar-Pedestrian(num).velocity)*dt_sim))^2 ...
                        -(norm(v_seniorcar-Pedestrian(num).velocity)*dt_sim)^2)/2;
                   w = lamda + (1 - lamda) * (1 + cos(theta_judge)) / 2;
                   g_fix = A*exp((0.2-b_alphabeta_fix)/B)*(norm(r) + norm(r-y_alphabeta))...
                        /(2*b_alphabeta_fix)*(1/2)*(r/norm(r)+(r-y_alphabeta)/norm(r-y_alphabeta));%seniorcar�̑傫�����g��
                  
                  f_alphabeta_seniorcar = w * g_fix;
                   
               end
           %�V�j�A�J�[�̑傫���l��

           %���s�҂��猩���Z�j�A�J�[�̕����i�O���[�o�����W�j
       end
               f_alphabeta_all = f_alphabeta_all + f_alphabeta_seniorcar;
           %end
       
           
    
     
       
        
        
 
    %% �͂̎Z�o�y�ё��x�̌v�Z
    f_social = f_goal + f_alphaB + f_alphabeta_all;

    
    Sf_ped = Pedestrian(num).velocity + f_social*dt_sim;
    %{
    if norm(Sf_ped) > 0.5*norm(Pedestrian(num).velocity) && norm(Sf_ped) <= norm(Pedestrian(num).velocity)
        Sf_ped = norm(Pedestrian(num).velocity)/norm(Sf_ped) * Sf_ped;
    end
    %}
    if norm(Sf_ped) > max_velocity
        Sf_ped = max_velocity/norm(Sf_ped) * Sf_ped;%maxvelocity�̑傫����Sf���������x�N�g��
    end
    %a = norm(Sf_ped)
    
    
end