% ŽÔ—¼‚Ì‰^“®—ÍŠw
function x = f_sim(x, u,dt)
% Motion Model

 
F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0];
 
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 0
    1 0
    0 1];
C = [0
    0
    dt
    0
    0];
w = u(1)*tan(u(2))/0.9; %ƒˆ[ƒŒƒCƒg‚ÌŽZou(1) = v,u(2) = delta,‘¬“x‚Æ‘€‘ÇŠp

if size(x,1) == 1
    x = x';
end
if size(u,1) == 1
    u = u';
end
x= F*x+B*u+C*w;
if x(3) > 2*pi
    x(3) = x(3) - 2*pi;
end
if x(3) < -2*pi
    x(3) = x(3) + 2*pi;
end