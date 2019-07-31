function existence_prohability_distribution_next_step = state_transition_model_ver11(x,predict_position_matrix,existence_prohability_distribution,Pedestrian,time,time_pre)
%�܂�����,�ȑO�̃o�[�W�������̓���m�F�����C��͊ϑ����ꂽ�ʒu�̗��������Ƃɕ��s�ґ��x���o������������������C2/27.1:05
%ver6�Ƃ̕ύX�_��existence_prohability_distribution�̃T�C�Y���������������̂ւ̑Ή��D���Ԃ��ƂɃf�[�^���ڍs����
%���Ԃ̃C���f�b�N�X���P�Ō��ݎ����ɂȂ�悤�ɒ����C�ϑ��덷�̕��̓}�[�W���ŋz������悤�ɒ���
%�����̕ύX��������
%�E�ӂɂ��Ĉ����̓Z�j�A�J�[�̈ʒu�C�ϑ��Ɋ�Â��ϑ����ꂽ���s�҂̑�\�ʒu�̋����Ɗp�x�C�ȒP�̂��߂ɕ��s�ҏ���p����
%�J�ڃ��f���ɂ͌��݂̑�����ۂƂ��������ƁC�i�s�������قȂ���̂��������Ƃ���������P�����������̂𔽉f������
%�ȒP�ɍ�邽�߂ɃV�~�����[�V������̃f�[�^��p���Ă���C����͏C������
%direction�g��Ȃ�����
%angle_increment = 0.25/180*pi;
%ave_range_error = -0.11;
%std_range_error = 0.2;
%ave_angle_error = -0.0066;
%std_angle_error = 0.046;
dt = 0.1;
%existence_prohability_distribution(k,x,y,t)�͊ϑ����ꂽ���s�҂̂���k�Ԗڂ̕��s��k��x�C���C���ɂ����鑶�݊m��
%predict_position_matrix�͍s�����ɕ��s�Ҕԍ��Ŋe�s�͕��s�Ҕԍ��C�S�[���̕����C���݂̃ƁC���C���݂�x�Cy�C0.1�b�O��x�Cy�D�D�D�C2�b�O��x�Cy
%NaN�͊ϑ��o���Ă��Ȃ��Ƃ�
ped_num = size(existence_prohability_distribution,4);
x_counter = size(existence_prohability_distribution,1);
y_counter = size(existence_prohability_distribution,2);
time_counter = size(existence_prohability_distribution,3);


%predict_position_matrix�͊ϑ����ꂽ���s�҂ɂ��āC���s�Ҕԍ��C�ʒu���C���C�i�s������2�����̒l
C_ = find(~isnan(predict_position_matrix(:,5)));%���ݎ����Ɋϑ����ꂽ���s�҂̃C���f�b�N�X.�����͋��o�[�W�����ƈꏏ�Ŗ��Ȃ�
ped_num2 = length(C_);%���ݎ����Ɋϑ����ꂽ���s�Ґ�
existence_prohability_distribution_next_step = ones(x_counter,y_counter,time_counter,ped_num)./(x_counter*y_counter);%��l�m���ŕ��z������
%size(existence_prohability_distribution_next_step)
%size(existence_prohability_distribution)
time_difference=time-time_pre;
if time_difference <= 40 && time_pre ~=0
    next_step_time_max_ind = 41-time_difference;
    last_step_time_min_ind = 1+time_difference;
    %last_step_time_max_ind = last_step_time_min_ind+(next_step_time_max_ind-1);
    existence_prohability_distribution_next_step(:,:,1:next_step_time_max_ind,:)=existence_prohability_distribution(:,:,last_step_time_min_ind:41,:);%�ȑO�̊m�����z�Ŏg����Ƃ���͎g��
end

% % for test by Yoshitake
% existence_prohability_distribution_next_step_new = existence_prohability_distribution_next_step;

for k = 1:ped_num%k�͂��ꂩ�瑶�݊m�����z�����߂���s�҂������C
    predict_position_array = predict_position_matrix(k,5:46);
    nearest_obseved_time_ind =find(~isnan(predict_position_array), 1 );
    if isempty(nearest_obseved_time_ind)%2�b�ȏ�ϑ�����
        continue
    end
    obseved_t = ceil(nearest_obseved_time_ind/2);
    time_difference = obseved_t-1;
    if time_difference >(time-time_pre)%�ȑO�̊m�����z�v�Z�ȍ~�V���Ȋϑ��������Ă��Ȃ�
        continue
    end
%    r_ped_to_ped = zeros(ped_num2,4);%�e���s�ҍ��W�ɂ�����ɍ��W��theta�C���C���x�̏�,k���ς�邽�тɏ�����
%% ���݊ϑ�������ꂽ���s�҂݂̂̏���
%{
    if predict_position_matrix(k,5) == time
        for ik = 1:ped_num2
        %��������predict_position_matrix�͍s�����ɕ��s�Ҕԍ����ɕ���ł��邪�C
        %�v�f�̈�ڂ����s�Ҕԍ��ɂȂ��Ă���D
        %C_�͌��ݎ����Ɋϑ����ꂽ���s�҂̃C���f�b�N�X���i�[����Ă���
        %���ݎ����ɂ����Ċϑ����ꂽ���s�ҊԂ̈ʒu�֌W���L�^
            if C_(ik)==k
                continue
            end
        
            r_ped_to_ped(ik,2)=norm(predict_position_matrix(C_(ik),2)-predict_position_matrix(k,2)...
                ,predict_position_matrix(C_(ik),3)-predict_position_matrix(k,3));
            r_ped_to_ped(ik,1)=atan2(predict_position_matrix(C_(ik),3)-predict_position_matrix(k,3),...
            predict_position_matrix(C_(ik),2)-predict_position_matrix(k,2));%�O���[�o�����W�n�ł̊p�x
            r_ped_to_ped(ik,3)=Pedestrian(predict_position_matrix(C_(ik),1)).velocity(1);
            r_ped_to_ped(ik,4)=Pedestrian(predict_position_matrix(C_(ik),1)).velocity(2);
        end

    end
%}
%% 0.5�b�O���猻�ݎ����̊ԂŊϑ�������ꂽ���s�҂̂݁@���̏����Ŋϑ�������ꂽ�����ƌ��ݎ����Ƃ̍���2�b�ȏ�̂��̂�continue���Ŕ����Ă���
    %predict_position_matrix(k,1)
    %k
    %r�ƃƂ�1�Ћ�Ԃ�4�_��x�Cy���W���o�����@�ϑ��덷�̓}�[�W���ŋz�����邩��p�[�e�B�N���̕���������
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
            21+round(now_pedestrian_position(o,2),1)*10];%existence_prohability_distribution_matrix�̃C���f�b�N�X�ƍ��킹���`��
    end
    
    x_max = max(now_ped_position_ind(:,1));
    x_min = min(now_ped_position_ind(:,1));
    x_num = x_max-x_min +1;
    y_max = max(now_ped_position_ind(:,2));
    y_min = min(now_ped_position_ind(:,2));
    y_num = y_max-y_min +1;
    Np = x_num*y_num;
    
    particle = zeros(Np,3);%�ʒu���C���Əd�݂��i�[
    for ix = 1:x_num
        for iy=1:y_num
            x_position = 0.1*(x_min-1+ix-1);
            y_position = -2+0.1*(y_min-1+iy-1);
            distance_to_particle = norm(x_position-(x(1)+0.9*cos(x(3))),y_position- (x(2)+0.9*sin(x(3))));%�ϑ��_�̓Z�j�A�J�[�̑O���̂���
            theta_to_particle =atan2(y_position- (x(2)+0.9*sin(x(3))),x_position-(x(1)+0.9*cos(x(3))))-x(3);
            observation_prohability_range = Gauss...
                (distance_to_particle,predict_position_matrix_for_simulator(k,2),std_range_error);%�ϑ�������ꂽ�ۂ́C����ʒu�̕��s�҂̑��݊m��(�������ɂ���)
            observation_prohability_angle = Gauss...
                (theta_to_particle,predict_position_matrix_for_simulator(k,1),std_angle_error);
            observation_prohability = observation_prohability_range*observation_prohability_angle;
            particle(y_num*(ix-1)+iy,:) = [x_position,y_position,observation_prohability];
        end
    end
    %�p�[�e�B�N���̏d�݂̌v�Z�܂ł͖��Ȃ�����
    %}
    %% �ϑ�������ꂽ���ԂƂ̑Ή��t�����s��
    for l = 1:time_counter%���b��܂Ō��邩�C(������0.1�b��܂�)
        %ver6�ŕ��s�҂̐i�s�������Ƃɏ����t������D
        
        min_i = max([1,floor(10*predict_position_matrix(k,nearest_obseved_time_ind+4))-ceil(2*(time_difference+l-1))]);%x����т���0.1m�ŃC���f�b�N�X���P�ς��
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
        %�ϑ���\�_�̗������g���ĕ��s�҂̑��x�\�����s��
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

        % ���R�[�h�̃R�����g�A�E�g by Yoshitake
%         tOrg = tic;
%        for i = min_i:max_i%�ϑ����ꂽ�_���琶�������p�[�e�B�N����x�͈̔́}2m���i1�b�Ԃ̐ݒ�̎��C���s�ґ��x����������1.7[m/s]�Ȃ��Ƃ��l����΁C�T���v�����O�ʒu�͂���ŏ\���j
%             for j = 1:41%�������͂��������}2m�����牽�����Ȃ�
%             
%                 sampling_position = [0.1*(i-1),-2+0.1*(j-1)];
%                 for ip = 1:Np
%                     r_position = sampling_position-particle(ip,1:2);
%                     velocity = r_position/((time-predict_position_matrix(k,5)+l)*dt);%�T���v�����O�����_�����b��̓_�Ƃ��邩�ő��x��ς���
%                     direction = atan2(r_position(2),r_position(1));
%                     %�������݊ϑ��������Ă��Ȃ����s�҂ɑ΂��đ��x���g���̂͂ǂ����Ǝv�����C�����̎��Ԃ��Ȃ��������߁C�܂��������Ă��Ȃ��D
%                     %�Z�j�A�J�[�̑��x�������𔽉f���Ă���Ƃ��āCx�i�S�j�𕽋ςƂ��Ĉ����̂���̈�
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
%                     %0.5�b�Ԃ̎������̈ʒu�A�ꂪ�W���΍�0.15m������C�p�x�̕W���΍���0.3/speed�Ƃ����D
%                     angle_prohability_2 = 1;
%                     
%                     velocity_norm_org(i-min_i+1,j,ip) = round(norm(velocity),5);
%                     direction_org(i-min_i+1,j,ip) = round(direction,5);
%                     speed_prohability_org(i-min_i+1,j,ip) = speed_prohability;
%                     angle_prohability_org(i-min_i+1,j,ip) = angle_prohability_1;
%                     
%                     % Comment out by Yoshitake
% %                     %1�͐i�s������ۂ��₷����������C2�͏�Q���ȊO�̕����ɐi�ސ�������
% %                     if predict_position_matrix(k,5) == time
% %                         [~,index]=min(abs(r_ped_to_ped(:,1)-direction)); 
% %                         r_v =  [velocity(1)-r_ped_to_ped(index,3),velocity(2)-r_ped_to_ped(index,4)];
% %                         r_p = [r_ped_to_ped(index,2)*cos(r_ped_to_ped(index,1)),r_ped_to_ped(index,2)*sin(r_ped_to_ped(index,1))];
% %                         if dot(r_p,r_v)/norm(r_v)>sqrt(norm(r_p)^2-0.4^2)%0.4�͕��s�ғ��m�̏Փˉ~���a�C�덷���l����Ȃ�0.4�ɉ���������K�v������
% %                             angle_prohability_2 = 0.5;%�Փˉ\�������������ɐi�ނƗ\�z����m���������邽�߂̏d�݁C�����͒������K�v
% %                         else
% %                             angle_prohability_2 = 1;
% %                         end
% %                     end
% 
%                     existence_prohability_distribution_next_step(i,j,time_counter-20+l,k) = existence_prohability_distribution_next_step(i,j,time_counter-20+l,k)...
%                         +particle(ip,3)*speed_prohability*angle_prohability_1*angle_prohability_2;
%                 end
%                 %sampling_position�m�F�̂��߂Ɏg����
%             end
%         end
%         %���K�������C�e���s�҂̊e�X�e�b�v���Ƃ̑��݊m���̘a���P�ɂȂ�悤�ɒ���
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