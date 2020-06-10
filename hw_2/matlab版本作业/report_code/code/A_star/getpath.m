function path=getpath(MYLIST,MYLIST_COUNT,a,b,xTarget,yTarget)


path(1:MYLIST_COUNT,:)=MYLIST(:,2:3);
path(MYLIST_COUNT+1,:)=[a,b];
path(MYLIST_COUNT+2,:)=[xTarget,yTarget];








