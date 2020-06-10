clear all;
clc;
p_x0=8;
v_x0=0;
a_x0=0;
j_x0=0;

p_y0=0;
v_y0=0;
a_y0=0;
j_y0=0;

p_z0=5;
v_z0=0;
a_z0=0;
j_z0=0;

K=30;%预测用点
dt=0.05;
K_disp = 800;% num of reference points K_disp*dt = 40
K_loop = 20000;%num of points for loop
w_p = 40;
w_v = 1;
w_a = 1;
log=[0 p_x0 v_x0 a_x0 j_x0 p_y0 v_y0 a_y0 j_y0 p_z0 v_z0 a_z0 j_z0];


ref_ind = 1;
for i = 1:1:K_loop
   t = i*dt;
   [Tpx,Tvx,Tax,Bpx,Bvx,Bax] = getPredictionMatrix(K,dt,p_x0,v_x0,a_x0); 
   [Tpy,Tvy,Tay,Bpy,Bvy,Bay] = getPredictionMatrix(K,dt,p_y0,v_y0,a_y0); 
   [Tpz,Tvz,Taz,Bpz,Bvz,Baz] = getPredictionMatrix(K,dt,p_z0,v_z0,a_z0);
   [x,y,z] = getReferenceXYZ2(ref_ind,dt,K,K_disp);
   
%    H = blkdiag(Tpx'*Tpx, Tpy'*Tpy,Tpz'*Tpz);
%    F = zeros(1,K*3); 
%    F(1,1:K) = Bpx'*Tpx-x'*Tpx;
%    F(1,K+1:2*K) = Bpy'*Tpy-y'*Tpy;
%    F(1,2*K+1:3*K) = Bpz'*Tpy-z'*Tpz;
   H = blkdiag(w_p*Tpx'*Tpx+Tvx'*Tvx+Tax'*Tax+eye(K), w_p*Tpy'*Tpy+Tvy'*Tvy+Tay'*Tay+eye(K)...
       ,w_p*Tpz'*Tpz+Tvz'*Tvz+Taz'*Taz+eye(K));
   F = zeros(1,K*3); 
   F(1,1:K) = w_p*(Bpx'*Tpx-x'*Tpx)+Bvx'*Tvx+Bax'*Tax;
   F(1,K+1:2*K) = w_p*(Bpy'*Tpy-y'*Tpy)+Bvy'*Tvy+Bay'*Tay;
   F(1,2*K+1:3*K) = w_p*(Bpz'*Tpy-z'*Tpz)+Bvz'*Tvz+Baz'*Taz;
   
   
   %% hard constraints
   Aieq = zeros(18*K,3*K);
   bieq = zeros(18*K,1);
   %%% x axis
   Aieq(1:K,1:K) = Tvx;
   bieq(1:K,1) = 6*ones(K,1) - Bvx;%v
   
   Aieq(K+1:2*K,1:K) = -Tvx;
   bieq(K+1:2*K,1) = 6*ones(K,1) + Bvx;%v
   
   Aieq(2*K+1:3*K,1:K) = Tax;
   bieq(2*K+1:3*K,1) = 3*ones(K,1) - Bax;%a
   
   Aieq(3*K+1:4*K,1:K) = -Tax;
   bieq(3*K+1:4*K,1) = 3*ones(K,1) + Bax;%a 
   
   Aieq(4*K+1:5*K,1:K) = eye(K);
   bieq(4*K+1:5*K,1) = 3*ones(K,1);%j 
   
   Aieq(5*K+1:6*K,1:K) = - eye(K);
   bieq(5*K+1:6*K,1) = 3*ones(K,1); %j
   %%% y axis
   Aieq(6*K+1:7*K,K+1:2*K) = Tvy;
   bieq(6*K+1:7*K,1) = 6*ones(K,1) - Bvy;%v
   
   Aieq(7*K+1:8*K,K+1:2*K) = -Tvy;
   bieq(7*K+1:8*K,1) = 6*ones(K,1) + Bvy;%v   
   
   Aieq(8*K+1:9*K,K+1:2*K) = Tay;
   bieq(8*K+1:9*K,1) = 6*ones(K,1) - Bay;%a  
   
   Aieq(9*K+1:10*K,K+1:2*K) = -Tay;
   bieq(9*K+1:10*K,1) = 6*ones(K,1) + Bay;%a 
   
   Aieq(10*K+1:11*K,K+1:2*K) = eye(K);
   bieq(10*K+1:11*K,1) = 3*ones(K,1); %j
   
   Aieq(11*K+1:12*K,K+1:2*K) = - eye(K);
   bieq(11*K+1:12*K,1) = 3*ones(K,1);%j    
   %%% z axis
   Aieq(12*K+1:13*K,2*K+1:3*K) = Tvz;
   bieq(12*K+1:13*K,1) = 6*ones(K,1) - Bvz;%v
   
   Aieq(13*K+1:14*K,2*K+1:3*K) = -Tvz;
   bieq(13*K+1:14*K,1) = ones(K,1) + Bvz;%v   
   
   Aieq(14*K+1:15*K,2*K+1:3*K) = Taz;
   bieq(14*K+1:15*K,1) = 3*ones(K,1) - Baz;%a   
   
   Aieq(15*K+1:16*K,2*K+1:3*K) = -Taz;
   bieq(15*K+1:16*K,1) = ones(K,1) + Baz;%a 
   
   Aieq(16*K+1:17*K,2*K+1:3*K) = eye(K);
   bieq(16*K+1:17*K,1) = 2*ones(K,1);%j 
   
   Aieq(17*K+1:18*K,2*K+1:3*K) = - eye(K);
   bieq(17*K+1:18*K,1) = 2*ones(K,1);%j  
   
   %% optimize
   J = quadprog(H,F,Aieq,bieq);
   
   jx=J(1);%只使用20次预测中的第一次，即4秒中的第0.2秒
   jy=J(K+1);
   jz=J(2*K+1);
   p_x0 = p_x0 + v_x0*dt + 0.5*a_x0*dt^2 + 1/6*jx*dt^3;
   v_x0 = v_x0 + a_x0*dt + 0.5*jx*dt^2;
   a_x0 = a_x0 + jx*dt;
   
   p_y0 = p_y0 + v_y0*dt + 0.5*a_y0*dt^2 + 1/6*jy*dt^3;
   v_y0 = v_y0 + a_y0*dt + 0.5*jy*dt^2;
   a_y0 = a_y0 + jy*dt;
   
   p_z0 = p_z0 + v_z0*dt + 0.5*a_z0*dt^2 + 1/6*jz*dt^3;
   v_z0 = v_z0 + a_z0*dt + 0.5*jz*dt^2;
   a_z0 = a_z0 + jz*dt;
   
   if abs(p_x0-x(1,1))<0.001 && abs(p_y0-y(1,1))<0.001 && abs(p_z0 - z(1,1))<0.001
      ref_ind = ref_ind + 1; 
   end
   if ref_ind > K_disp
       break;
   end
   log = [log; t p_x0 v_x0 a_x0 jx p_y0 v_y0 a_y0 jy p_z0 v_z0 a_z0 jz];  
    
end
% figure;
% plot(log(:,1),log(:,2),'b');hold on;
% plot(log(:,1),log(:,5),'r');hold on;
% plot(log(:,1),log(:,8),'g');hold on;



%%%%%%%%%% get reference
w = 0.628;
v = -0.5;
x = zeros(K_disp,1);
y = zeros(K_disp,1);
z = zeros(K_disp,1);

for i = 1:1:K_disp

    x(i,1) = v/2*(i*dt)*cos(w*i*dt) ;
    y(i,1) = v/2*(i*dt)*sin(w*i*dt) ;
    z(i,1) = v*(i*dt) ;
end

%%%%%%%%%% plot position in 2D
% figure;
% subplot(3,1,1);
% plot(log(:,1),x(:,1),'.','Color','b');hold on;
% plot(log(:,1),log(:,2),'r');hold on;
% 
% subplot(3,1,2);
% plot(log(:,1),y(:,1),'.','Color','b');hold on;
% plot(log(:,1),log(:,6),'r');hold on;
% 
% subplot(3,1,3);
% plot(log(:,1),z(:,1),'.','Color','b');hold on;
% plot(log(:,1),log(:,10),'r');hold on;
%% plot position in 3D
figure;
plot3(x,y,z,'.','Color','b');hold on;
plot3(log(:,2),log(:,6),log(:,10),'r','linewidth',2);hold on;

%% plot velocity and acceleration 
figure;
subplot(3,1,1);
plot(log(:,1),log(:,3),'r');hold on;%v
plot(log(:,1),log(:,4),'b');hold on;%a
plot(log(:,1),log(:,5),'g');hold on;%j
legend({'v','a','j'},'Location','northeast');
title('x axis');

subplot(3,1,2);
plot(log(:,1),log(:,7),'r');hold on;
plot(log(:,1),log(:,8),'b');hold on;
plot(log(:,1),log(:,9),'g');hold on;
legend({'v','a','j'},'Location','northeast');
title('y axis');

subplot(3,1,3);
plot(log(:,1),log(:,11),'r');hold on;
plot(log(:,1),log(:,12),'b');hold on;
plot(log(:,1),log(:,13),'g');hold on;
legend({'v','a','j'},'Location','northeast');
title('z axis');
