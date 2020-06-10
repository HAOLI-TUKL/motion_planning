function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = zeros(3, n_all_poly);
    beq_start = zeros(3, 1);
    
    beq_start(1,1) = start_cond(1);%p
    beq_start(2,1) = start_cond(2);%v
    beq_start(3,1) = start_cond(3);%a 

    Aeq_start(1,1) = 1;
    Aeq_start(2,1) = -n_order;
    Aeq_start(2,2) = n_order; 
    Aeq_start(3,1) = n_order*(n_order-1);
    Aeq_start(3,2) = -2*n_order*(n_order-1);     
    Aeq_start(3,3) = n_order*(n_order-1);     
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = zeros(3, n_all_poly);
    beq_end = zeros(3, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    %
    %
    %
    %

    beq_end(1,1) = end_cond(1);%p
    beq_end(2,1) = end_cond(2);%v
    beq_end(3,1) = end_cond(3);%a

    Aeq_end(1,n_all_poly) = 1;
    Aeq_end(2,n_all_poly-1) = -n_order;
    Aeq_end(2,n_all_poly) = n_order; 
    Aeq_end(3,n_all_poly-2) = n_order*(n_order-1);
    Aeq_end(3,n_all_poly-1) = -2*n_order*(n_order-1);     
    Aeq_end(3,n_all_poly) = n_order*(n_order-1);  
    
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments

    
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    %
    %
    %
    %

    for seg = 1:(n_seg-1)
        Aeq_con_p(seg,seg*(n_order+1))=1;
        Aeq_con_p(seg,seg*(n_order+1)+1)=-1;
        
    end

    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments

    
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    %
    %
    %
    %

    for seg = 1:(n_seg-1)
       Aeq_con_v(seg,seg*(n_order+1)-1) = -n_order;
       Aeq_con_v(seg,seg*(n_order+1)) = n_order;
       Aeq_con_v(seg,seg*(n_order+1)+1) = n_order;
       Aeq_con_v(seg,seg*(n_order+1)+2) = -n_order;
    end

    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments

    
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    %
    %
    %
    %

    for seg = 1:(n_seg-1)
       Aeq_con_a(seg,seg*(n_order+1)-2) = n_order*(n_order-1);
       Aeq_con_a(seg,seg*(n_order+1)-1) = -2*n_order*(n_order-1);
       Aeq_con_a(seg,seg*(n_order+1)) = n_order*(n_order-1);
       
       Aeq_con_a(seg,seg*(n_order+1)+1) = -n_order*(n_order-1);
       Aeq_con_a(seg,seg*(n_order+1)+2) = 2*n_order*(n_order-1);
       Aeq_con_a(seg,seg*(n_order+1)+3) = -n_order*(n_order-1);
    end
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end