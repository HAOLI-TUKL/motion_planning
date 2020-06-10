clear all;
clc;
p_0=10;
v_0=0;
a_0=0;
K=20;% size of J 
dt=0.2;% K*dt = 4s,每次优化往前预测20次共4秒 
log=[0 p_0 v_0 a_0 ];
w1 = 10;
w2 = 1;
w3 = 1;
w4 = 1;

for t = 0.2:0.2:10%50次 10s
    
   [Tp,Tv,Ta,Bp,Bv,Ba] = getPredictionMatrix(K,dt,p_0,v_0,a_0); 
   H = w4*eye(K) + w1*Tp'*Tp + w2*Tv'*Tv + w3*Ta'*Ta ;
   F = w1*Bp'*Tp + w2*Bv'*Tv + w3*Ba'*Ta;
   
   J=quadprog(H,F,[],[]);
   
   j=J(1);%只使用20次预测中的第一次，即4秒中的第0.2秒
   p_0 = p_0 + v_0*dt + 0.5*a_0*dt^2 + 1/6*j*dt^3;
   v_0 = v_0 + a_0*dt + 0.5*j*dt^2;
   a_0 = a_0 + j*dt;
   
   log = [log;t p_0 v_0 a_0];  
    
end

plot(log(:,1),log(:,2),'b');hold on;
plot(log(:,1),log(:,3),'r');hold on;
plot(log(:,1),log(:,4),'g');hold on;





