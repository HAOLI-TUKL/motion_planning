function [x,y,z] = getReferenceXYZ(K,dt,t)
% w = 0.08/pi*180;
 w = 0.628;
v = -0.5;
x = zeros(K,1);
y = zeros(K,1);
z = zeros(K,1);
start = ceil(t/dt);



for i = start:K+start-1

    x(i-start+1,1) = v/2*(i*dt)*cos(w*i*dt) ;
    y(i-start+1,1) = v/2*(i*dt)*sin(w*i*dt) ;
    z(i-start+1,1) = v*(i*dt) ;
end
