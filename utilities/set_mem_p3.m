function [x_next,x_next_zonotope]=set_mem_p3(fstate ,Q,G,x,x_zonotope)

%[f, F_bar]= jaccsd(fstate,x);
%u = f - F_bar*x;
%x_next = F_bar*x + u;
%x_next_zonotope = zonotope(F_bar*x_zonotope); % to be tested

[f, F_bar]= jaccsd(fstate,x);
x_next = F_bar*x ;
x_next_zonotope = zonotope(F_bar*x_zonotope); % to be tested

%correct center
% temp = x_next_zonotope.Z;
% temp(:,1)=x_next;
% x_next_zonotope=zonotope(temp);

%%----------
P_next = F_bar*P*transpose(F_bar) + G*Q*transpose(G);

%P_next = F_bar*P*conj(F_bar) + G*Q*conj(G);
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

