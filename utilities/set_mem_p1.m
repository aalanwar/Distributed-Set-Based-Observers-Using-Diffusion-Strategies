function [eita,eita_zonotope]=set_mem_p1(method,x,P,hmeas,Rl,yl,x_zonotope)



for i=1:length( Rl)
    [ h(i), H_cap(i,:)]= jaccsd(hmeas{i},x);
end
   sigma = Rl{1};
if strcmp(method,'diffKF')
    
    sum1=0;
    for i=1:length( Rl)
       % [ h, H_cap]= jaccsd(hmeas{i},x);
        sum1 = sum1 + conj(H_cap(i,:))'*( Rl{i}^-1)*H_cap(i,:);
    end
    sum1 = sum1 + (P)^-1 ;
    P_next=(sum1)^-1;
    for i=1:length( Rl)
        %[ h, H_cap]= jaccsd(hmeas{i},x);
        lambda(:,i)=P_next*conj(H_cap(i,:))'*Rl{i}^(-1);
        %lambda1(:,i)= H*H'*H_cap' / (H_cap*(H*H')*H_cap' + sigma.^2) ;
        %lambda2(:,i)= H*H'*H_cap' / (length( Rl)*H_cap*(H*H')*H_cap' + length( Rl)*sigma.^2);
    end
    
elseif strcmp(method,'frobenius') || strcmp(method,'svd') || strcmp(method,'volume')
    P_next=0;
 
    H = generators(x_zonotope);
    sigma = Rl{1};
    
    %x0=zeros(24,8);
    x0=zeros(2,length(Rl));
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    %lb = -6*ones(1,length(x0));
    %ub = 6*ones(1,length(x0));
    lb = [];
    ub = [];
    nonlcon = @circlecon;
   
    %options = optimoptions(@fmincon,'Algorithm','active-set','Display','off');
    options = optimoptions(@fmincon,'Algorithm', 'sqp', 'TolX', 1e-9, 'TolFun', 1e-9,'Display','off');
    
    lambda = fmincon(@fun,x0,A,b,Aeq,beq,lb,ub,nonlcon, options);
    
end


sum1=0;
for i=1:length( Rl)
    % [ h, H_cap]= jaccsd(hmeas{i},x);
    sum1 = sum1 + lambda(:,i)*( yl{i} - H_cap(i,:)*x );
end

%s_zonotope = reduce(s_zonotope,'girard',30);
eita = sum1 + x;
%eita_zonotope = x_zonotope + zonotope(P_next * s_zonotope) ;
%eita_zonotope = x_zonotope + P_next * sum ;
p_new = eita;


H_new = eye(length(x));
H = generators(x_zonotope);
for i=1:length( Rl)
    %[ h, myH_Cap]= jaccsd(hmeas{i},x);%hmeas{i} myh
    H_new = H_new - lambda(:,i)*H_cap(i,:);
    new_gen(:,i) = sigma*lambda(:,i);
end
H_new = H_new * H;


H_new2 = [H_new new_gen];
x_zonotope = zonotope([p_new H_new2]);
%x_zonotope = reduce(x_zonotope,'girard',3);
%correct centers of zonotopes
%%temp = x_zonotope.Z;
%%temp(:,1) = eita;
eita_zonotope = x_zonotope;


%return to old value
%eita_zonotope = x_zonotope_old;
% %-- Diffusion update
% x = eita;
% %-- Time update
% [f F_bar]= jaccsd(fstate,x);
% u = f - F_bar*x;
% x_next = F_bar*x + u;
% P_next = F_bar*P_next*transpose(F_bar) + G*Q*transpose(G);



    function nfro = fun(lambda)
        argH_new = eye(length(x));
        H = generators(x_zonotope);
        for i=1:length( Rl)
            %[ h, myH_Cap]= jaccsd(hmeas{i},x);%hmeas{i} myh
            argH_new = argH_new - lambda(:,i)*H_cap(i,:);
            argnew_gen(:,i) = sigma*lambda(:,i);
        end
        argH_new = argH_new * H;
        argH_new2 = [argH_new argnew_gen];
%         sum11=0;
%         for i=1:length( Rl)
%             % [ h, H_cap]= jaccsd(hmeas{i},x);
%             sum11 = sum11 + lambda(:,i)*( yl{i} - h(i));
%         end
%         argmineita = sum11 + x;
%        argminzono = zonotope([zeros(length(x)) argH_new2]);

        if strcmp(method,'frobenius')
            nfro = norm(argH_new2,'fro');
        elseif strcmp(method,'svd')
            nfro = sum(svd(argH_new2));     
        elseif strcmp(method,'volume')
            %zono = zonotope([x H_new2]);
            nfro = volume(argminzono);
        end
    end

    function [c,ceq] = circlecon(x)
        c=[];
        %c = norm( x(stateSize*N+1:end,1),1)-numberofattacks;
        %c=[];
        %c=[];
        ceq =[];
        
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