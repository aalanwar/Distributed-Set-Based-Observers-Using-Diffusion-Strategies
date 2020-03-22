function [x_next_zonotope]=set_mem_p3(fstate ,Q,x_zonotope)


[f, F_bar]= jaccsd(fstate,x_zonotope.center);
new_center = F_bar*x_zonotope.center ;
new_gen = [F_bar*x_zonotope.generators, Q];
x_next_zonotope = zonotope([new_center, new_gen]); % to be tested


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

