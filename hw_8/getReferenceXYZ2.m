function [x_r,y_r,z_r] = getReferenceXYZ2(ref_index,dt,K,K_disp)
w = 0.628;
v = -0.5;
% K = 800;% K*dt = 40s 
% dt = 0.05;
x = zeros(K_disp,1);
y = zeros(K_disp,1);
z = zeros(K_disp,1);
x_r = zeros(K,1);
y_r = zeros(K,1);
z_r = zeros(K,1);
for i = 1:1:K_disp

    x(i,1) = v/2*(i*dt)*cos(w*i*dt) ;
    y(i,1) = v/2*(i*dt)*sin(w*i*dt) ;
    z(i,1) = v*(i*dt) ;
end
for j = 1:1:K
    x_r(j,1) = x(ref_index,1);
    y_r(j,1) = y(ref_index,1);
    z_r(j,1) = z(ref_index,1);

end

% plot3(x(:,1),y(:,1),z(:,1),'r');
