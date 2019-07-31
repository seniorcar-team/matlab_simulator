% 操作可能範囲かの判定（減速機構がついているため6kmで大きく曲がらないようになっている）
function possible = possible_velocity(u_target)
possible = 1;
x1 = deg2rad(16.7);
y1 = 6/3.6;
x2 = deg2rad(38.3);
y2 = 4.5/3.6;
a = (y2-y1)/(x2-x1);
b = y1-x1*a;
if u_target(1) > 4.5/3.6 %4.5km/h以下ならok
    if abs(u_target(2)) > deg2rad(16.7) %16.7deg以下ならok
        if u_target(1) > a*abs(u_target(2))+b
            possible = 0;
        end
    end
end
