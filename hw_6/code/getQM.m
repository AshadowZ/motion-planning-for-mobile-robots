function [Q, M] = getQM(n_seg, n_order, ts)
    Q = [];
    M = [];
    M_k = getM(n_order);
    d_order = (n_order + 1)/2;
    for k = 1:n_seg
        %#####################################################
        % STEP 2.1 calculate Q_k of the k-th segment 
        Q_k = [];
        t_k = 1;
        s_k = ts(k);    % 归一化，直接参考paper里的公式
        Q_k = zeros(n_order + 1, n_order + 1);
        for i = d_order:n_order
            for j = d_order:n_order
                Q_k(i+1,j+1) = factorial(i)/factorial(i-d_order)*...
                    factorial(j)/factorial(j-d_order)/...
                    (i+j-n_order)*t_k^(i+j-n_order)/s_k^(2*d_order - 3);
            end
        end
        Q = blkdiag(Q, Q_k); % Q和之前的求法差不多，通过映射矩阵M变成Bezier的对应值
        M = blkdiag(M, M_k);
    end
end