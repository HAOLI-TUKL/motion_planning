function M = getM(n_seg, n_order, ts)
    M = [];
    for k = 1:n_seg
        M_k = [];
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        %
        %
        %
        %
        M_k(1,1) = 1;
        M_k(2,2) = 1;
        M_k(3,3) = 2;
        M_k(4,4) = 6;
        for i = 1:(n_order+1)
            M_k(5,i) = ts(k,1)^(i-1);
        end
        for i = 1:n_order
            M_k(6,i+1) = i*ts(k,1)^(i-1);
        end
        for i = 1:(n_order-1)
            M_k(7,i+2) = i*(i+1)*ts(k,1)^(i-1);
        end
        for i = 1:(n_order-2)
            M_k(8,i+3) = i*(i+1)*(i+2)*ts(k,1)^(i-1);
        end
        M = blkdiag(M, M_k);
    end
end