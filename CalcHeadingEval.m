%% headingの評価関数を計算する関数
function heading=CalcHeadingEval(x,goal)

theta=Pi2Pi(x(3));%ロボットの方位
goalTheta=atan2(goal(2)-x(2),goal(1)-x(1));%ゴールの方位

targetTheta=abs(Pi2Pi(goalTheta-theta));%ゴールまでの方位差分[deg]

heading=pi-targetTheta;