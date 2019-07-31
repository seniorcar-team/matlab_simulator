%距離を入力し，センサ位置と歩行者重心を結んだ線とセンサを通る歩行者円に接する直線のなす角を出力
function angle = occlusion_angle(distance)

a = 0.3/sqrt(distance^2-0.04);%歩行者を半径0.2mの円として近似したとき，センサ位置（0，0），歩行者（0，d）にいるとして円の接線の傾きを求めた，
if isreal(a)
    angle = atan2(a,1);
else angle = pi/2;
end
end
