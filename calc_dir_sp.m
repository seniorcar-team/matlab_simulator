function dir_sp = calc_dir_sp(x,Pedestrian,xt)
%DWA-CSC�p�̕]���֐��C�p�x�Ɋւ��锱����^����
%���̊֐��̕Ԃ�l�Ƀƃ}�b�N�X��������C�ƃ}�b�N�X�͕��̒l�H
dir_sp =0;
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
        theta = (pi-abs(Pi2Pi(x(3)-atan2(Pedestrian(i).velocity(2),Pedestrian(i).velocity(1)))))/pi;%����Η��������ƒႢ�l�C�_�����ł�DWA�͓��������̎��Ɉ���������₷���Ƃ̘b���������̂ŁC�}�������ǂꂾ�������������邩�̌W���H
    else
        continue;
    end    
    dir_sp = dir_sp+theta*Pi2Pi(xt(3)-atan2(Pedestrian(i).velocity(2),Pedestrian(i).velocity(1)));%���_���ł͂����Ȃ��Ă����ǕЕ��̊p�x�ɂ������K�����ĂȂ��̂ƁC�Ȃ��p���΂̎��]�����Œ�ɂȂ�œ��ꂵ�Ă��Ȃ��̂���c
end

