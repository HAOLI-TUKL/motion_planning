function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    %
    %
    %
    %
    %
    Ct = [];
    
    
    Ct_start = [];
    Ct_start(1,1) = 1;
    Ct_start(2,2) = 1;
    Ct_start(3,3) = 1;
    Ct_start(4,4) = 1;
    
    Ct_wp_pos_total = zeros(1,n_seg - 1);
    for seg = 1:(n_seg - 1)
        Ct_wp_pos = zeros(8,n_seg - 1);
        Ct_wp_pos(1,seg) = 1;
        Ct_wp_pos(5,seg) = 1;
        Ct_wp_pos_total = [Ct_wp_pos_total;Ct_wp_pos];
    end
    Ct_wp_pos_res=Ct_wp_pos_total(2:8*(n_seg - 1)+1,1:(n_seg - 1));
    
    
    
    Ct_end = [];
    Ct_end(1,1) = 1;
    Ct_end(2,2) = 1;
    Ct_end(3,3) = 1;
    Ct_end(4,4) = 1;
        
    
    Ct_wp_other_total = zeros(1,3*(n_seg - 1));
    for seg = 1:(n_seg - 1)
        Ct_wp_other = zeros(8,3*(n_seg - 1));
        Ct_wp_other(2,1+3*(seg-1)) = 1;
        Ct_wp_other(3,2+3*(seg-1)) = 1;
        Ct_wp_other(4,3+3*(seg-1)) = 1;
        Ct_wp_other(6,1+3*(seg-1)) = 1;
        Ct_wp_other(7,2+3*(seg-1)) = 1;
        Ct_wp_other(8,3+3*(seg-1)) = 1;
        Ct_wp_other_total = [Ct_wp_other_total;Ct_wp_other];
    end
    Ct_wp_other_res=Ct_wp_other_total(2:8*(n_seg - 1)+1,1:3*(n_seg - 1));
    
   Ct = zeros(8*n_seg,4*n_seg+4);
   Ct(1:4,1:4) = Ct_start;
   Ct(5:4+8*(n_seg-1),5:4+(n_seg-1)) = Ct_wp_pos_res;
   Ct(4+8*(n_seg-1)+1:8*n_seg,4+(n_seg-1)+1:4+(n_seg-1)+1+3) = Ct_end;
   Ct(5:4+8*(n_seg-1),4+(n_seg-1)+4+1:4*n_seg+4) = Ct_wp_other_res;
    
%     Ct(1,1) = 1;
%     Ct(2,2) = 1;
%     Ct(3,3) = 1;
%     Ct(4,4) = 1;

%     
%     Ct(5,5) = 1;
%     Ct(9,5) = 1;
%     Ct(13,6) = 1;
%     Ct(17,6) = 1;      
% 
%     
%     Ct(21,7) = 1;
%     Ct(22,8) = 1;
%     Ct(23,9) = 1;
%     Ct(24,10) = 1;
%  
%     Ct(6,11) = 1;
%     Ct(7,12) = 1;
%     Ct(8,13) = 1;
%     Ct(10,11) = 1;
%     Ct(11,12) = 1;
%     Ct(12,13) = 1;
%     
%     Ct(14,14) = 1;
%     Ct(15,15) = 1;
%     Ct(16,16) = 1;
%     Ct(18,14) = 1;
%     Ct(19,15) = 1;
%     Ct(20,16) = 1;
        
end