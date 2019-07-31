function vel_sp = calc_vel_sp(x,Pedestrian,xt)
%DWA�|CSC�Ɋւ���]���֐��C���x�Ɋւ��锱����^����D
%���݂̏Փˉ\���̂���Ώۂ����x��ۂ����܂ܐi�ނƂ��C���傤�ǏՓ˂���Ƃ��̑��x���ő�Ƃ��āC����ɋ߂��قǔ������傫��
%���̊֐��̕Ԃ�l�Ƀȃ}�b�N�X��������
vel_sp = 0;
for i = 1:length(Pedestrian)
    Pedestrian_position = rot([Pedestrian(i).tmp_position(1)-x(1),Pedestrian(i).tmp_position(2)-x(2)],-x(3));%�ԗ����W�n(���݂̎ԗ��̈ʒu�y�юp���ɕ��s�ړ��Ɖ�]������������)
    xt_position = rot([xt(1)-x(1),xt(2)-x(2)],-x(3));
    Pedestrian_direction = atan2(Pedestrian(i).velocity(2),Pedestrian(i).velocity(1))-x(3);
    Pedestrian_velocity = rot([Pedestrian(i).velocity(1),Pedestrian(i).velocity(2)],-x(3));%�ԗ����W�n(���݂̎ԗ��̈ʒu�y�юp���ɕ��s�ړ��Ɖ�]������������)
    CSC = 0;
    %�Փˉ\������
    if Pedestrian_position(1) < 0
        continue;
    end
    if Pedestrian_position(2) > 0
        if atan2(-Pedestrian_position(2),-Pedestrian_position(1))...
                <= Pedestrian_direction && ...
                Pedestrian_direction < 0
            CSC =1;
        else
            continue;
        end
    else
        if 0 <= Pedestrian_direction && ...
            atan2(-Pedestrian_position(2),-Pedestrian_position(1)) > Pedestrian_direction
        CSC =1;
        else
            continue;
        end
    end
    if CSC ==1
        gradient = tan(atan2(Pedestrian(i).velocity(2),Pedestrian(i).velocity(1))-x(3));
        D = abs(gradient*(xt_position(1)-Pedestrian_position(1))-1*(xt_position(2)-Pedestrian_position(2)))...
            /sqrt(gradient^2+1^2);%���s�҂̐i�s�����Ɋ�Â���������̋���
        vel_max = norm([Pedestrian(i).velocity(1),Pedestrian(i).velocity(2)])*...
            (-1*Pedestrian_position(2)/(tan(Pedestrian_direction))+Pedestrian_position(1))...
            /(-1*Pedestrian_position(2)/(tan(Pedestrian_direction))*sqrt(1+tan(Pedestrian_direction)^2));%�Փ˓_�ɕ��s�҂Ɠ����^�C�~���O�œ��鑬�x
        vel_sp = vel_sp+D*abs(Pedestrian_velocity(2))/vel_max;%���_���ł͂����Ȃ��Ă����Ǌ����Ă���̂����{�b�g�̍ō����e���x�Ŋ�����̂����s�҂̑��x�̓��쉄����ւ̐������������Ȃ̂���c
    else 
        continue;
    end
end

