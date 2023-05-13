function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = [];
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment 
        %
        fac = @(x) x*(x-1)*(x-2)*(x-3); % 包装个函数吧
        Q_k = zeros(n_order+1,n_order+1); % 未知数个数=次数+1
        for i = 0:n_order % 直接使用课件中给的式子
            for l = 0:n_order
                if (i < 4) || (l < 4) % 对应i>=4且l>=4
                    continue;
                else
                    Q_k(i + 1,l + 1) = fac(i) * fac(l) / (i + l - 7) * ts(k)^(i + l -7); 
                end
            end
        end
        Q = blkdiag(Q, Q_k); % 得到Q，即得到J目标函数
    end
end