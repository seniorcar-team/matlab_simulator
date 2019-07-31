function occupancy_matrix = caluculate_occupanted_area(x,Pedestrian)
ped_num = length(Pedestrian);
A = zeros(ped_num,3);
pedestrian_radius = 0.2;%������s�҂̐�߂�̈�ɂ΂������������Ȃ�occulision_angle�̂Ƃ���̊֐���ύX����K�v������D
occupancy_matrix = zeros(2,721);
for i = 1:ped_num
    if isreal(Pedestrian(i).tmp_position(1)) == false
        Pedestrian(i).tmp_position(1)
    end

    
    [theta_global,range] = cart2pol(Pedestrian(i).tmp_position(1) - (x(1)+0.9*cos(x(3)))...
        ,Pedestrian(i).tmp_position(2) - (x(2)+0.9*sin(x(3))));
    theta = theta_global - x(3);
    A(i,:) = [i,range,theta];
end

A = sortrows(A,2,'descend');

angle_increment = (0.25*pi/180);
for num = 1:ped_num
    
    d_theta = occlusion_angle(A(num,2));
    theta_max = A(num,3)+d_theta;
    theta_min = A(num,3)-d_theta;
    if theta_min > pi/2
        continue
    elseif theta_max < -pi/2
        continue;
    end
    
    
    max_ind = min(721,floor((theta_max + pi/2)/angle_increment)+1);
    min_ind = max(1,ceil((theta_min + pi/2)/angle_increment)+1);
    
    for ind = min_ind:max_ind
        occupancy_matrix(1,ind) = A(num,1);
    end
end
for ind_num = 1:721
    if occupancy_matrix(1,ind_num) == 0
        continue;
    end
    observed_pedestrian_num = find(A(:,1)==occupancy_matrix(1,ind_num));%A�̍s�񒆂̉��s�ڂɊϑ����ꂽ���s�҂�����̂��T��
    phai = -pi/2+angle_increment*(ind_num-1)-A(observed_pedestrian_num,3);%�������𑪂낤�Ƃ��Ă�������ƕ��s�Ғ��S�����Ƃ̂Ȃ��p
    a = tan(phai);
    d = A(observed_pedestrian_num,2);
    X = (d - sqrt(d^2 -(a^2+1)*(d^2-pedestrian_radius^2)))/(a^2+1);
    if imag(X)~=0
        occupancy_matrix(1,ind_num) = 0;
        continue
    end
    Y = a*X;
    occupancy_matrix(2,ind_num) = sqrt(X^2+Y^2)+randn*0.03;%�Z���T�̃m�C�Y�������Ă���
end

    
    
