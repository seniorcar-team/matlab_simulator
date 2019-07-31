%velŠÖ”
function vel=CalcVelEval(vt)
global desired_speed;
if vt <= desired_speed
    vel = vt;
elseif vt > desired_speed
    vel = 2* desired_speed - vt;
end
