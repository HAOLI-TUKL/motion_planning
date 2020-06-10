function y = goal_exist(expended_list,xTarget,yTarget)
y=0;
for i =1:8
    if expended_list(i,1)==xTarget && expended_list(i,2)==yTarget
        y=1;
    end
end


