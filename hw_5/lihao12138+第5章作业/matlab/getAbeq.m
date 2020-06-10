function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    %
    %
    %
    %
   
    beq_start(1,1) = start_cond(1);%p
    beq_start(2,1) = start_cond(2);%v
    beq_start(3,1) = start_cond(3);%a
    beq_start(4,1) = start_cond(4);%j

    Aeq_start(1,1) = 1;%p
    Aeq_start(2,2) = 1;%v
    Aeq_start(3,3) = 2;%a
    Aeq_start(4,4) = 6;%j





    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    %
    %
    %
    %

    beq_end(1,1) = end_cond(1);%p
    beq_end(2,1) = end_cond(2);%v
    beq_end(3,1) = end_cond(3);%a
    beq_end(4,1) = end_cond(4);%j

    for i = 0:n_order
        Aeq_end(1,n_all_poly-i) = ts(n_seg)^(n_order-i);
    end
    for i = 0:(n_order-1)
        Aeq_end(2,n_all_poly-i) = (n_order-i)*ts(n_seg)^(n_order-i-1);
    end
    for i = 0:(n_order-2)
        Aeq_end(3,n_all_poly-i) = (n_order-i-1)*(n_order-i)*ts(n_seg)^(n_order-i-2);
    end
    for i = 0:(n_order-3)
        Aeq_end(4,n_all_poly-i) = (n_order-i-2)*(n_order-i-1)*(n_order-i)*ts(n_seg)^(n_order-i-3);
    end
    
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    %
    %
    %
    %
   
    for seg = 1:(n_seg-1)
        for i = 1:(n_order+1) 
            Aeq_wp(seg ,i + (seg-1)*(n_order+1)) = ts(seg)^(i-1);
        end
        beq_wp(seg,1) = waypoints(seg+1,1);
    end 

    
    
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    %
    %
    %
    %
    for seg = 1:(n_seg-1)
        for i = 1:(n_order+1)
            Aeq_con_p(seg,i+(n_order+1)*(seg-1)) = ts(seg)^(i-1);
        end
        Aeq_con_p(seg,(n_order+1)*seg+1)=-1;
    end

    
    %#####################################################
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    %
    %
    %
    %
    for seg = 1:(n_seg-1)
        for i = 1:n_order
            Aeq_con_v(seg,i+(n_order+1)*(seg-1)+1) = i*ts(seg)^(i-1);
        end
        Aeq_con_v(seg,(n_order+1)*seg+2)=-1;
    end
    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    %
    %
    %
    %
    for seg = 1:(n_seg-1)
        for i = 1:(n_order-1)
            Aeq_con_a(seg,i+(n_order+1)*(seg-1)+2) = i*(i+1)*ts(seg)^(i-1);
        end
        Aeq_con_a(seg,(n_order+1)*seg+3)=-2;
    end   
    
    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    %
    %
    %
    %
    for seg = 1:(n_seg-1)
        for i = 1:(n_order-2)
            Aeq_con_j(seg,i+(n_order+1)*(seg-1)+3) = i*(i+1)*(i+2)*ts(seg)^(i-1);
        end
        Aeq_con_j(seg,(n_order+1)*seg+4)=-6;
    end  
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end