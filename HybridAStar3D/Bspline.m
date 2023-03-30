function [X,Y,Z] = Bspline(x,y,z,k) 
    % % 定义控制点
    % x = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115];
    % y = [0, 0, 0, -1, -3, -4, -6, -8, -11, -12, -11, -11, -11, -11, -12, -13, -15, -18, -19, -21, -23, -22, -24, -24];
    % z = [-1, -2, -3, -4, -3, -2, -1, -1, -6, -8, -10, -6, -4, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2];
     
    n=length(x)-1;% B样条曲线有n+1个控制点
    % k=4;% 三次B样条，因此阶数=次数+1
     
    % 用于储存生成的点
    X=[];
    Y=[];
    Z=[];
     
    Nik_u=zeros(1,n+1);% 用于储存基函数
    nodevector=U_quasi_uniform(n,k);% 生成节点向量
     
    a = 10*(n+1);

    for u=0:1/a:1-0.01% 循环次数决定组成B样条曲线点的总数量
        for j=1:n+1% 生成Bi,3这一系列基函数
            Nik_u(j)=BaseFunction(j,k,u,nodevector);
        end
        % 由于B样条曲线具有局部支撑性，因此只有u周围一定区间会参与到计算
        X=[X,x*Nik_u'];
        Y=[Y,y*Nik_u'];
        Z=[Z,z*Nik_u'];
    end
    plot3(X,Y,Z)
end
    % 生成准均匀B样条曲线
function nodevector=U_quasi_uniform(n, k)
    nodevector=zeros(1,n+k+1);% 节点数=控制点的个数+阶数
    piecewise = n - k + 2;% B样条曲线的段数=控制点个数-次数
    if piecewise==1% 只有一段曲线时，直接末尾重复度k
        nodevector(n+1+1:n+k+1)=1;
    else
        for i=1:n-k+1% 中间段内节点均匀分布：两端共2k个节点，中间还剩(n+k+1-2k=n-k+1）个节点
            nodevector(k+i)=nodevector(k+i-1)+1/piecewise;
        end
        nodevector(n+1+1:n+k+1)=1;% 末尾重复度k
    end
end

%定义第i个k阶B样条基函数
function Nik_u=BaseFunction(i, k, u, NodeVector)
    if k==1% 定义Bi,0这一系列基函数
        if u>=NodeVector(i)&&u<NodeVector(i+1)
            Nik_u=1;
        else
            Nik_u=0;
        end
    else
        % 公式中的两个分母
        denominator_1 = NodeVector(i + k - 1) - NodeVector(i);
        denominator_2 = NodeVector(i + k) - NodeVector( i + 1);
        % 如果遇到分母为0的情况，定义为1以便继续计算
        if denominator_1==0
            denominator_1=1;
        end
        if denominator_2==0
            denominator_2=1;
        end
        % 不断递归
        Nik_u=(u - NodeVector(i)) / denominator_1 * BaseFunction(i, k - 1, u, NodeVector) ...
        + (NodeVector(i + k) - u) / denominator_2 * BaseFunction(i + 1, k - 1, u, NodeVector);
    end
end