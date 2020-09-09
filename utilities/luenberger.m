function [res_zonotope]=luenberger(z1,hl,Rl,yl,F,varargin)


if nargin==5
    %The optimization function is based on norm of the generators
    method='normGen';
elseif nargin==6
    method =varargin{1};
end

    H = generators(z1);
if strcmp(method,'svd') || strcmp(method,'radius') 
    lambda0=zeros(length(z1.center),length(Rl));
    options = optimoptions(@fminunc,'Algorithm', 'quasi-newton','Display','off');
    %find the weights
    lambda = fminunc(@fun,lambda0, options);
elseif strcmp(method,'normGen')
    % Find the analytical solution
    h_combined=[];
    for i=1:length(hl)
        h_combined = [ h_combined ; hl{i}];
    end    
    gamma=eye(length(hl));
    num= F*H*H'*h_combined';
    den = h_combined * H*H' * h_combined' ;
    for i=1:length(hl)
        den = den + gamma(:,i) *Rl{i}^2* gamma(:,i)';
    end
    
    lambda = num * den^-1;
else
    disp('Method is not supported');
    return;
end



%prepare center
c_new=F*z1.center;
for i=1:length( Rl)
    c_new = c_new + lambda(:,i)*( yl{i} - hl{i}*z1.center );
end

%prepare generators
part1 = F;%eye(length(z1.center));
for ii=1:length(Rl)
    part1 = part1 - lambda(:,ii)*hl{ii};
    part2(:,ii) = Rl{ii}*lambda(:,ii);
end
part1 = part1 * H;
H_new = [part1 part2];
res_zonotope = zonotope([c_new H_new]);



    function nfro = fun(lambda)
        part1 = eye(length(z1.center));
        for ii=1:length(Rl)
            part1 = part1 - lambda(:,ii)*hl{ii};
            part2(:,ii) = Rl{ii}*lambda(:,ii);
        end
        part1 = part1 * H;
        H_new = [part1 part2];
        if strcmp(method,'svd')
            nfro = sum(svd(H_new));
        elseif strcmp(method,'radius')
            nfro = radius(zonotope([zeros(length(z1.center),1) H_new]));
        end
        
    end


end
