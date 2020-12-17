function [eita,P_next]=dif_ekf_p1(x,P,hmeas,Rl,yl)
% diffusion Kalman
%   Detailed explanation goes here


P_minus = P;
sum=0;
for i=1:length( Rl)
   H_cap= hmeas{i}; 
  sum = sum + conj(H_cap)'*( Rl{i}^-1)*H_cap;
end
sum = sum + (P)^-1 ;
P_next=(sum)^-1;

sum=0;
for i=1:length( Rl)
    H_cap= hmeas{i};
  sum = sum + conj(H_cap)'*Rl{i}^(-1)*( yl{i} - H_cap*x);
end
eita = P_next*sum + x;



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