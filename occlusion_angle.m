%��������͂��C�Z���T�ʒu�ƕ��s�ҏd�S�����񂾐��ƃZ���T��ʂ���s�҉~�ɐڂ��钼���̂Ȃ��p���o��
function angle = occlusion_angle(distance)

a = 0.3/sqrt(distance^2-0.04);%���s�҂𔼌a0.2m�̉~�Ƃ��ċߎ������Ƃ��C�Z���T�ʒu�i0�C0�j�C���s�ҁi0�Cd�j�ɂ���Ƃ��ĉ~�̐ڐ��̌X�������߂��C
if isreal(a)
    angle = atan2(a,1);
else angle = pi/2;
end
end
