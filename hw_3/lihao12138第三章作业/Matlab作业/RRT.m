%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% 流程初始化
clear all; close all;
x_I=1; y_I=1;           % 设置初始点
x_G=700; y_G=700;       % 设置目标点
Thr=50;                 %设置目标点阈值
Delta= 30;              % 设置扩展步长
%% 建树初始化
T.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).dist=0;          %从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;     %
%% 开始构建树——作业部分
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%地图x轴长度
yL=size(Imp,2);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% 绘制起点和目标点
count=1;
for iter = 1:3000
    x_rand=[];
    %Step 1: 在地图中随机采样一个点x_rand
    %提示：用（x_rand(1),x_rand(2)）表示环境中采样点的坐标
    x_rand(1) = randi(xL);
    x_rand(2) = randi(xL);
    
    x_near=[];
    %Step 2: 遍历树，从树中找到最近邻近点x_near 
    %提示：x_near已经在树T里
    distance=[];
    for i = 1:count
      distance(i) = sqrt((T.v(i).x - x_rand(1))^2 + (T.v(i).y - x_rand(2))^2);   
    end
    pos = find(distance==min(distance));
    x_near(1) = T.v(pos).x;
    x_near(2) = T.v(pos).y;
   

    x_new=[];
    %Step 3: 扩展得到x_new节点
    %提示：注意使用扩展步长Delta
    k = (x_rand(2) - x_near(2))/(x_rand(1) - x_near(1)) ;
    b = x_rand(2) - k*x_rand(1);
    A = k^2 + 1 ; 
    B = 2*b*k - 2*x_near(1) - 2*k*x_near(2);
    C = x_near(1)^2 - 2*b*x_near(2)+ x_near(2)^2 + b^2 - Delta^2;
    
    if x_near(1) ~= x_rand(1) 
        if B^2 - 4*A*C > 0
            x1 = (-B - sqrt(B^2 - 4*A*C))/2/A;
            y1 = k*x1+b;
            x2 = (-B + sqrt(B^2 - 4*A*C))/2/A;
            y2 = k*x2+b;
            dist1 = sqrt((x1-x_rand(1))^2 + (y1-x_rand(2))^2);
            dist2 = sqrt((x2-x_rand(1))^2 + (y2-x_rand(2))^2);
            if dist1 > dist2
                 x_new(1) = x2;
                 x_new(2) = y2;
            else
                 x_new(1) = x1;
                 x_new(2) = y1;             
            end
            
        end  
    else
        if x_near(2) > x_rand(2)
           x_new(1) = x_near(1);
           x_new(2) = x_near(1) - Delta;
           
        else
           x_new(1) = x_near(1);
           x_new(2) = x_near(1) + Delta;  
        end
    
 
              
    end

    
    
    
    
    %检查节点是否是collision-free
    if ~collisionChecking(x_near,x_new,Imp) 
       continue;
    end
    count=count+1;
    
    %Step 4: 将x_new插入树T 
    %提示：新节点x_new的父节点是x_near
   
    T.v(count).x = x_new(1);        
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = x_near(1);     
    T.v(count).yPrev = x_near(2);
    T.v(count).dist= sqrt((x_I-x_new(1))^2 + (y_I-x_new(2))^2);          
    T.v(count).indPrev = pos; 
    %Step 5:检查是否到达目标点附近 
    %提示：注意使用目标点阈值Thr，若当前节点和终点的欧式距离小于Thr，则跳出当前for循环
    if sqrt((x_G-x_new(1))^2 + (y_G-x_new(2))^2) < Thr
        break;
    end
    
    
    
   %Step 6:将x_near和x_new之间的路径画出来
   %提示 1：使用plot绘制，因为要多次在同一张图上绘制线段，所以每次使用plot后需要接上hold on命令
   %提示 2：在判断终点条件弹出for循环前，记得把x_near和x_new之间的路径画出来
   plot([x_near(1) x_new(1) ],[x_near(2) x_new(2)],'linewidth',5,'color','g'); hold on;
%    plot(x_rand(1),x_rand(2),'o');hold on;
   
   pause(0.1); %暂停0.1s，使得RRT扩展过程容易观察
end
%% 路径已经找到，反向查询
if iter < 2000
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % 终点加入路径
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end


