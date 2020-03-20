function [x_zonotope]=kalmanInspired(fstate,method,Q,hl,Rl,yl,x_zonotope,zonol)
%papaer title: Kalman-inspired distributed set-membership observers
[f, F_bar]= jaccsd(fstate,x_zonotope.center);
[ h, H_cap]= jaccsd(hl{1},x_zonotope.center);

%paper notations
%C = H_cap;
%A = F_bar;

% tempContZono = conZonotope(x_zonotope);
% intersFlag = 0;
% for i=1:length(zonol)
%         if isIntersecting(zonol{i},tempContZono)
%             tempContZono = tempContZono & conZonotope(zonol{i});
%             intersFlag =1;
%         end
% end

tempContZono = x_zonotope;
intersFlag = 0;
for i=1:length(zonol)
       % if isIntersecting(zonol{i},tempContZono)
            tempContZono = tempContZono & zonol{i};
       %     intersFlag =1;
      %  end
end

%if intersFlag ==1
%    x_zono = zonotope(tempContZono);
%else
    x_zono = tempContZono;
%end

Px= x_zono.generators*x_zono.generators';
Pv = Rl{1}*Rl{1}';
%%%%%%%%%%%%%%%%%%%
% x0=zeros(2,1);
% A = [];
% b = [];
% Aeq = [];
% beq = [];
% %lb = -6*ones(1,length(x0));
% %ub = 6*ones(1,length(x0));
% lb = [];
% ub = [];
% nonlcon = @circlecon;
% options = optimoptions(@fmincon,'Algorithm', 'sqp', 'TolX', 1e-9, 'TolFun', 1e-9,'Display','off');
% L1 = fmincon(@funmin,x0,A,b,Aeq,beq,lb,ub,nonlcon, options);

%%%%%%%%%%%%%%%%%%%%
L  = F_bar*Px*H_cap'*(H_cap*Px*H_cap'+Pv)^-1;
%L=L1;
newCenter = F_bar*x_zono.center + L*(yl{1} - H_cap * x_zono.center);
newGen = [(F_bar - L *H_cap)*x_zono.generators, Q , -L*Rl{1}];
x_zonotope = zonotope([newCenter newGen]);



function nfro = funmin(L_var)
newGen = [(F_bar - L_var *H_cap)*x_zono.generators, Q , -L_var*Rl{1}];
nfro =  norm(newGen,'fro');
end

function [c,ceq] = circlecon(L_var)
c=[];
ceq =[];%sum(lambda)-1;

end

end
function [z,A]=jaccsd(fun,x)
% JACCSD Jacobian through complex step differentiation
% [z J] = jaccsd(f,x)
% z = f(x)
% J = f'(x)
% example :
% f=@(x)[x(2);x(3);0.05*x(1)*x(2)];
% [x,A]=jaccsd(f,[1 1 1])
%
% x =
%
%     1.0000
%     1.0000
%     0.0500
%
%
% A =
%
%          0    1.0000         0
%          0         0    1.0000
%     0.0500    0.0500         0
z=fun(x);
n=numel(x);%Number of elements in an array or subscripted array expression.
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    x1=x;
    x1(k)=x1(k)+h*i;
    A(:,k)=imag(fun(x1))/h;
end
end

