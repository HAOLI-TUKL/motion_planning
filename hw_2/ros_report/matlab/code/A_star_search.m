function path = A_star_search(map,MAX_X,MAX_Y)
%%
    
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    OPEN(OPEN_COUNT,1)=0;
%     CLOSED_COUNT=CLOSED_COUNT+1;
%     CLOSED(CLOSED_COUNT,1)=xNode;
%     CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;
    path = [];
    aa=0;
   
    MYLIST=[];
    MYLIST_COUNT=0;
    
    
    
%     CLOSED=[1 1;1 2;2 1;2 2;3 1;1 3;2 3;3 2];
%     CLOSED_COUNT=8;
%     
    
    
    
    
    
%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while(1) %you have to dicide the Conditions for while loop exit 

    expended_list=[OPEN(1,2)-1,OPEN(1,3);
                   OPEN(1,2)-1,OPEN(1,3)-1;
                   OPEN(1,2)-1,OPEN(1,3)+1;
                   OPEN(1,2)+1,OPEN(1,3);
                   OPEN(1,2)+1,OPEN(1,3)-1;
                   OPEN(1,2)+1,OPEN(1,3)+1;
                   OPEN(1,2),OPEN(1,3)+1;
                   OPEN(1,2),OPEN(1,3)-1];

    if goal_exist(expended_list,xTarget,yTarget)
        path = getpath(MYLIST,MYLIST_COUNT,OPEN(1,2),OPEN(1,3),xTarget,yTarget);
        disp('path found');
        break;
    end

    disp('expended_list');
    disp(expended_list);
    xNode=OPEN(1,2);
    yNode=OPEN(1,3);
 %   goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=OPEN(1,7);
    WAIT_COUNT=0;


    
     for i =1:8
        in_closed =0;
        for j = 1:CLOSED_COUNT
            if expended_list(i,1) == CLOSED(j,1) && expended_list(i,2) == CLOSED(j,2)
                in_closed =1;%flag =1 means that this one meets the preconditions
            end
        end
        if in_closed ~=1 && 0 < expended_list(i,1)&&expended_list(i,1)< MAX_X && 0< expended_list(i,2)&&expended_list(i,2) < MAX_Y 
            goal_distance=distance(expended_list(i,1),expended_list(i,2),xTarget,yTarget);
            path_cost_i=path_cost+1;
            f=goal_distance+path_cost_i;
            WAIT_COUNT=WAIT_COUNT+1;
            waiting_list(WAIT_COUNT,:)=[expended_list(i,:) goal_distance path_cost_i f];   
        end
    end
       
    
    
    
    
    
    
%     disp('waiting_list');
%     disp(waiting_list);
%     [WAIT_COUNT,a]=size(waiting_list);
    if WAIT_COUNT ~= 0
        for i=1:WAIT_COUNT
            any_same = 0;
            for j= 1:OPEN_COUNT
                %%if there is same (x,y), then dont add into OPEN
               if waiting_list(i,1) == OPEN(j,2) && waiting_list(i,2) == OPEN(j,3) 
                   if  waiting_list(i,5) < OPEN(j,8)
                       OPEN(j,8)=waiting_list(i,5);
                       OPEN(j,7)=waiting_list(i,4);
                       OPEN(j,6)=waiting_list(i,3);
                       OPEN(j,5)=yNode;
                       OPEN(j,4)=xNode;
                   end
                   any_same =1;
               end
            end
            if any_same ~=1
               OPEN_COUNT=OPEN_COUNT+1;
               OPEN(OPEN_COUNT,:)=insert_open(waiting_list(i,1),waiting_list(i,2),xNode,yNode,waiting_list(i,3),waiting_list(i,4),waiting_list(i,5)); 
            end
        end 
    end
    disp('OPEN');
    disp(OPEN);   

    aa=aa+1;
    disp(aa);
    

    if OPEN_COUNT == 1
        disp('no path found');
        break;
    end 
    
    
    MYLIST_COUNT=MYLIST_COUNT+1;
    MYLIST(MYLIST_COUNT,:)=OPEN(1,:);
    OPEN=OPEN(2:OPEN_COUNT,:);
    OPEN_COUNT=OPEN_COUNT-1;
    OPEN=bubsort(OPEN,OPEN_COUNT);
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    disp('OPEN_after');
    disp(OPEN);
    disp('OPEN_after');
    disp(OPEN_COUNT);
    disp('MYLIST');
    disp(MYLIST); 
    disp('CLOSED');
    disp(CLOSED);        
    
     %
     %finish the while loop
     %
     
    end %End of While Loop
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node.This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
    
   
end
