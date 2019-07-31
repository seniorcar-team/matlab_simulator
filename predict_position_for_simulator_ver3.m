function predict_position_matrix_abs_cart = predict_position_for_simulator_ver3(occupancy_matrix,predict_position_matrix_abs_cart_pre,x_seniorcar,Pedestrian)
%��\�ʒu�̃J�[�e�V�A���̐�΍��W�Ɗϑ����ꂽ�������L�^����Ă���
%���s�Ҕԍ��C�����C���݂̃ƁC���C���݂�x�Cy�C0.1�b�O��x�Cy�D�D�D�C2�b�O��x�Cy
ped_num = size(predict_position_matrix_abs_cart_pre,1);
predict_position_matrix_abs_cart = zeros(ped_num,46);
predict_position_matrix_abs_cart(:,7:46) = predict_position_matrix_abs_cart_pre(:,5:44);%�ЂƂO�̂��͍̂�����݂���0.1�b�O�̈ʒu�ɂȂ邩��2���炵�Ĉڂ�
angle_increment = 0.25/180*pi;
for i = 1:ped_num
    A = occupancy_matrix(1,:);%�p�x���Ƃ̊ϑ����ꂽ���s�҂��i�[����Ă���
    observed_ind_array = find(A == i);
    if abs(deg2rad(Pedestrian(i).direction))<pi/2
        direction = 1;%�Z�j�A�J�[�Ɠ��������ɃS�[���������Ă���ƍl����C�摜�x�[�X�Ői�s�������ǂ̌�������񌳓I�ɍl���邱�Ƃ͉\
    else
        direction = 0;
    end
    
    if isempty(observed_ind_array)
        predict_position_matrix_abs_cart(i,1:6)  = [i,direction,NaN,NaN,NaN,NaN];
        continue
    
    elseif occupancy_matrix(2,observed_ind_array(1))*angle_increment*(length(observed_ind_array)-1)< 0.1
        %�����~�ϑ��ł����p�x�͈̔͂̑傫�����ϑ��ł��������̒���
        predict_position_matrix_abs_cart(i,1:6)  = [i,direction,NaN,NaN,NaN,NaN];
        continue
    end
    observable_point_num = length(observed_ind_array);
    x = zeros(1,observable_point_num);
    y = zeros(1,observable_point_num);
    for j = 1:observable_point_num        
        [x(j),y(j)] = pol2cart(-pi/2+angle_increment*(min(observed_ind_array)+(j-1)-1),occupancy_matrix(2,min(observed_ind_array)+j-1));
    end
    
    C0 = [mean(x),mean(y)]; 
    f = @(C) norm((x-C(1)).^2 + (y-C(2)).^2-0.2^2);
    Cf1 = fminsearch(f,C0);%��\�_
    if isreal(Cf1(1))~= 1
        predict_position_matrix_abs_cart(i,1:6)  = [i,direction,NaN,NaN,NaN,NaN];
        continue
    end
    if isreal(Cf1(2))~= 1
        predict_position_matrix_abs_cart(i,1:6)  = [i,direction,NaN,NaN,NaN,NaN];
        continue
    end
    [representative_theta,representative_range]=cart2pol(Cf1(1),Cf1(2));
    [represented_x,represented_y]=...
            pol2cart(representative_theta+x_seniorcar(3),representative_range);

        
    predict_position_matrix_abs_cart(i,1:6) = ...
    [i,direction,representative_theta,representative_range,x_seniorcar(1)+0.9*cos(x_seniorcar(3))+represented_x...
    ,x_seniorcar(2)+0.9*sin(x_seniorcar(3))+represented_y];
end
