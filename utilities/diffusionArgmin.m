function [ x_zonotope ] = diffusionArgmin( c,x_zonotope,zonol,eita )
%DIFFUSIONARGMIN Summary of this function goes here
%   Detailed explanation goes here

x0=(1/length(c))*ones(length(c),1);
A = [];
b = [];
Aeq = [];
beq = [];
%lb = -6*ones(1,length(x0));
%ub = 6*ones(1,length(x0));
lb = [];
ub = [];
nonlcon = @circlecon;
options = optimoptions(@fmincon,'Algorithm', 'sqp', 'TolX', 1e-9, 'TolFun', 1e-9,'Display','off');
lambda = fmincon(@fun,x0,A,b,Aeq,beq,lb,ub,nonlcon, options);

cen = lambda(1)*x_zonotope.center;
gen = lambda(1)*x_zonotope.generators;
for i =2:length(c)
    gen = [gen lambda(i)*zonol{i-1}.generators];
    cen = cen + lambda(i)*zonol{i-1}.center;
end
cen = cen/sum(lambda);
gen = (1/sum(lambda))*gen;
x_zonotope = zonotope([cen,gen]);



    function nfro = fun(lambda)
        %cen_inter = lambda(1)*x_zonotope.center;
        gen_inter = lambda(1)*x_zonotope.generators;

        for i =2:length(c)
            gen_inter = [gen_inter lambda(i)*zonol{i-1}.generators];
           % cen_inter = cen_inter + lambda(i)*zonol{i-1}.center;
           
        end
        gen_inter = (1/sum(lambda))*gen_inter;
        %cen_inter = (1/sum(lambda))*cen_inter;
       % z_avgarg=zonotope([cen_inter,gen_inter]);
        %nfro =  radius(z_avgarg);
        %nfro =  volume(z_avgarg);
        %nfro =  norm2center(z_avgarg,'fro');
        nfro =  norm(gen_inter,'fro');
    end

%avg  = (1/(lambda(1)+lambda(2)+lambda(3))) * [lambda(1)*g1 lambda(2)*g2 lambda(3)*g3];
%nfro =  norm(avg,'fro');
% nfro = sum(svd(avg));
    function [c,ceq] = circlecon(lambda)
        % c is less than zero c<=0
        %c = norm( x(stateSize*N+1:end,1),1)-numberofattacks;
        %c=[];
        c=[];
%          gen_inter = lambda(1)*x_zonotope.generators;
%         sumlambda = lambda(1);
%         for j =2:length(zonol)
%             gen_inter = [gen_inter lambda(j)*zonol{j-1}.generators];
%             sumlambda = sumlambda + lambda(j);
%         end
%         gen_inter = (1/sumlambda)*gen_inter;
%         index =1;
%         for j =1:length(zonol)
%         c(index) = norm(gen_inter,'fro') - norm(zonol{j}.generators,'fro')  ;
%         c(index+1) = -1 *  lambda(j);
%         index = index+2;
%         end
        
        ceq =[];%sum(lambda)-1;
        
    end
end

