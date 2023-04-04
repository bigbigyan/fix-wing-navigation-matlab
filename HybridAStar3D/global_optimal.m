% function optimal_waypoints = optimal()
%     fun = @(x) sum((x(:,1).^2 + x(:,2).^2 + x(:,3).^2).^2);
% 
%     x0 = rand(100,3);
% 
%     options = optimoptions('fminunc','Algorithm','quasi-newton','Display','iter',MaxIterations=1000);
%     [x_opt, fval, exitflag, output] = fminunc(fun,x0,options)
% 
% end

function x_opt = global_optimal(waypoints)
    x0 = waypoints;

    fun = @fun_fs;
    options = optimoptions("fminunc",'Algorithm','quasi-newton',HessianApproximation="lbfgs",MaxIterations=1000);
    [x_opt, fval ,exitflag, output] = fminunc(fun,x0,options);

end

function f = fun_fs(x)
    n = size(x,1);
    pb = 3;
    fs = 0;
    fv = 0;
    fa = 0;

    for i = pb-1:n-pb+1
        fs_1 = (norm(x(i+1,:)+x(i-1,:)-x(i,:)-x(i,:)))^2;
        fs = fs + fs_1;
    end

    for i = 1:n-1
        v(i,:) = x(i+1,:) - x(i,:);
    end
    for i = n-1
        if norm(v(i,:),2) > 2
            fv = fv + norm(v(i,:),2) - 2;
        end
    end

    for i = 1:n-2
        a(i,:) = v(i+1,:) - v(i,:);
    end
    for i = n-2
        if norm(a(i,:),2) > 2
            fa = fa + norm(a(i,:),2) - 2;
        end
    end

    f = fs + fv + fa;
end