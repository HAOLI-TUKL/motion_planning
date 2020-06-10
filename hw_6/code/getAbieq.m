function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 3.2.1 p constraint


    %besides the start point and the end point of each segment,upper limit
    Aieq_p_1 = zeros(n_seg*6,n_all_poly);
    bieq_p_1 = zeros(n_seg*6,1);
    for seg = 1:n_seg
        for order = 1:(n_order-1)
            Aieq_p_1(order+(seg-1)*6,order+1+(seg-1)*8)=1; 
        end
        for i = 1:6
            bieq_p_1(i+(seg-1)*6,1) = corridor_range(seg,2);
        end
    end
    %besides the start point and the end point of each segment,lower limit
    Aieq_p_2 = zeros(n_seg*6,n_all_poly);
    bieq_p_2 = zeros(n_seg*6,1);
    for seg = 1:n_seg
        for order = 1:(n_order-1)
            Aieq_p_2(order+(seg-1)*6,order+1+(seg-1)*8)=-1; 
        end
        for i = 1:6
            bieq_p_2(i+(seg-1)*6,1) = -corridor_range(seg,1);
        end
    end    
    %start points and the end points of each segment,upper limit
    Aieq_p_3 = zeros(n_seg-1,n_all_poly);
    bieq_p_3 = zeros(n_seg-1,1); 
    for seg = 1:n_seg-1
        Aieq_p_3(seg,8*seg)=1;
        bieq_p_3(seg,1)=min(corridor_range(seg,2),corridor_range(seg+1,2));
    end
    %start points and the end points of each segment,lower limit
    Aieq_p_4 = zeros(n_seg-1,n_all_poly);
    bieq_p_4 = zeros(n_seg-1,1); 
    for seg = 1:n_seg-1
        Aieq_p_4(seg,8*seg)=-1;
        bieq_p_4(seg,1)=-max(corridor_range(seg,1),corridor_range(seg+1,1));
    end  
    %combine
    Aieq_p=[Aieq_p_1;Aieq_p_2;Aieq_p_3;Aieq_p_4];
    bieq_p=[bieq_p_1;bieq_p_2;bieq_p_3;bieq_p_4];
    

    %#####################################################
    % STEP 3.2.2 v constraint   

    % upper velocity limit
    Aieq_v_upper = zeros(n_seg*n_order,n_all_poly);
    bieq_v_upper = zeros(n_seg*n_order,1);
    
    Aieq_v_block = zeros(n_order,n_order+1);
    for i = 1:n_order
       Aieq_v_block(i,i) = - n_order;
       Aieq_v_block(i,i+1) = n_order;      
    end
    
    for i = 1:n_seg
        Aieq_v_upper((i-1)*n_order+1:i*n_order,(i-1)*(n_order+1)+1:i*(n_order+1)) = Aieq_v_block;
    end
    
    for i = 1:n_seg*n_order
        bieq_v_upper(i,1) = v_max;
    end
    
    % lower velocity limit
    Aieq_v_lower = zeros(n_seg*n_order,n_all_poly);
    bieq_v_lower = zeros(n_seg*n_order,1);    
    
    Aieq_v_block = zeros(n_order,n_order+1);
    for i = 1:n_order
       Aieq_v_block(i,i) = n_order;
       Aieq_v_block(i,i+1) = - n_order;      
    end
    for i = 1:n_seg
        Aieq_v_lower((i-1)*n_order+1:i*n_order,(i-1)*(n_order+1)+1:i*(n_order+1)) = Aieq_v_block;
    end
    for i = 1:n_seg*n_order
        bieq_v_lower(i,1) = v_max;
    end
    
    %combine
    Aieq_v = zeros(2*n_seg*n_order,n_all_poly);
    bieq_v = zeros(2*n_seg*n_order,1);
    
    Aieq_v(1:n_seg*n_order,1:n_all_poly) = Aieq_v_upper;
    Aieq_v(n_seg*n_order+1:2*n_seg*n_order,1:n_all_poly) = Aieq_v_lower;
    bieq_v(1:n_seg*n_order,1) = bieq_v_upper;
    bieq_v(n_seg*n_order+1:2*n_seg*n_order,1) = bieq_v_lower;

    

    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = zeros(2*n_seg*(n_order-1),n_all_poly);
    bieq_a = zeros(2*n_seg*(n_order-1),1);
    % upper acce limit
    Aieq_a_upper = zeros(n_seg*(n_order-1),n_all_poly);    
    Aieq_a_block = zeros(n_order-1,n_order+1);
    for i = 1:n_order-1
       Aieq_a_block(i,i) = n_order * (n_order-1);
       Aieq_a_block(i,i+1) = -2*n_order * (n_order-1);  
       Aieq_a_block(i,i+2) = n_order * (n_order-1);
    end
    for i = 1:n_seg
        Aieq_a_upper((i-1)*(n_order-1)+1:i*(n_order-1),(i-1)*(n_order+1)+1:i*(n_order+1)) = Aieq_a_block;
    end

    % lower acce limit
    Aieq_a_lower = zeros(n_seg*(n_order-1),n_all_poly);    
    Aieq_a_block = zeros(n_order-1,n_order+1);
    for i = 1:n_order-1
       Aieq_a_block(i,i) = -n_order * (n_order-1);
       Aieq_a_block(i,i+1) = 2*n_order * (n_order-1);  
       Aieq_a_block(i,i+2) = -n_order * (n_order-1);
    end
    for i = 1:n_seg
        Aieq_a_lower((i-1)*(n_order-1)+1:i*(n_order-1),(i-1)*(n_order+1)+1:i*(n_order+1)) = Aieq_a_block;
    end
    
    %combine
    Aieq_a(1:n_seg*(n_order-1),1:n_all_poly) = Aieq_a_upper;
    Aieq_a(n_seg*(n_order-1)+1:2*n_seg*(n_order-1),1:n_all_poly) = Aieq_a_lower;
    
    for i = 1:2*n_seg*(n_order-1)
        bieq_a(i,1) = a_max; 
    end
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v;Aieq_a];
    bieq = [bieq_p; bieq_v;bieq_a];


end