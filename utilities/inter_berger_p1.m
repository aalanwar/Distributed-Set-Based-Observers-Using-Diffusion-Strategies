function [eita,eita_zonotope]=inter_berger_p1(method,x,hmeas,Rl,yl,x_zonotope,F_bar,Q)
% diffusion Kalman
%   Detailed explanation goes here


H = generators(x_zonotope);
x0=ones(2,length(Rl));%zeros(24,8);
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
nonlcon = @circlecon;



%options = optimset(@fminsearch,'Display','off','MaxFunEvals',0.1);
%options = optimoptions(@fmincon,'Algorithm', 'sqp', 'TolX', 1e-9, 'TolFun', 1e-9,'Display','off');
%lambda = fmincon(@fun_berger,x0,A,b,Aeq,beq,lb,ub,nonlcon, options);
options = optimset('MaxFunEvals',10000,'MaxIter',1e4);
lambda = fminsearch(@fun_berger,x0,options);
%calculate the center
sum1=F_bar;
sum2=0;
for i=1:length( Rl)
    sum1 = sum1 - lambda(:,i)* hmeas{i};
    sum2 = sum2 + lambda(:,i)*yl{i};
end

eita = sum1 * x +sum2;

H_new = F_bar; % should be F
for i=1:length( Rl)
    H_new = H_new - lambda(:,i) *hmeas{i};
    R_gen(:,i) = -Rl{1}*lambda(:,i);
end
H_new = H_new * H;
H_new2 = [H_new R_gen Q]; 
               
x_zonotope = zonotope([eita H_new2]);

eita_zonotope = x_zonotope;


    function nfro = fun_berger(lamb)
        H_p1 = F_bar;
        for i=1:length( Rl)
            H_p1 = H_p1 - lamb(:,i)*hmeas{i};
            R_gene(:,i) = -Rl{1}*lamb(:,i);
        end
        H_p1 = H_p1 * H;
        H_total = [H_p1 R_gene Q];%Q=0.002*eye(length(x))
        
        if strcmp(method,'frobenius')
            nfro = norm(H_total,'fro');
        elseif strcmp(method,'svd')
            nfro = sum(svd(H_total));
        else
            disp(strcat(method,' is Not supported!'));
        end
        
    end
function [c,ceq] = circlecon(x)
    c=[];
ceq =[];

end
end
