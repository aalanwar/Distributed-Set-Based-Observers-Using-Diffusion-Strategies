function [Z] = andAveraging1(zonol,varargin)
% andAveraging - computes the intersection between list of zonotopes
% options: methods to calculate the weights
% 'normGen'
% 'volume'
% 'radius'
%
%
% Syntax:
%    Z = and(zonol,options)
%
% Inputs:
%    zonol - list of zonotopes
%    options - one of the options (default 'normGen')
% Outputs:
%    Z - zonotope object enclosing the intersection
%
% Example:
%    zonol{1} = zonotope([2 2 2;1 2 0]);
%    zonol{2} = zonotope([3 1 -1 1;3 1 2 0]);
%    zonol{3} = zonotope([1 3 -1 1;2 3 -2 0]);
%
%
%    res = andAveraging(zonol);
%
%    figure
%    hold on
%     plot(zonol{1},[1,2],'r');
%     plot(zonol{2},[1,2],'r-+');
%     plot(zonol{3},[1,2],'r-*');
%    plot(res,[1,2],'k');
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: none

% Author: Amr Alanwar
% Written: 9-Feb-2020
% Last update: 9-Mar-2020 (add analytical solution to normGen)
%
% Last revision: ---

%------------- BEGIN CODE --------------
%check if they are intersecting
intersectionFlag=true;
% for ii=1:length(zonol)
%     for j=ii+1:length(zonol)
%         if ~isIntersecting(zonol{ii},zonol{j})
%             intersectionFlag=false;
%         end
%     end
% end

if intersectionFlag==false
    disp('There is no visible intersection.');
    return;
end
%2 inputs
if nargin==1
    %The optimization function is based on norm of the generators
    method='normGen';
    %3 inputs
elseif nargin==2
    method =varargin{1};
end


if strcmp(method,'normGen')
    tVec = zeros(1,length(zonol));
    w = zeros(1,length(zonol));
    %find Analytical solution
    invtVecSum = 0;
    for ii=1:length(zonol)
        tVec(ii)=trace(zonol{ii}.generators*zonol{ii}.generators');
        invtVecSum = invtVecSum + 1/tVec(ii);
    end
    
    for ii=1:length(zonol)
        w(ii)= 1/(tVec(ii) * invtVecSum );
    end
elseif  strcmp(method,'volume') || strcmp(method,'radius')
    w0=(1/length(zonol))*ones(length(zonol),1);
    options = optimoptions(@fminunc,'Algorithm', 'quasi-newton','Display','off');
    %find the weights
    w = fminunc(@fun,w0, options);
else
    disp('option is not supported.');
    return;
end

cen = w(1)*zonol{1}.center;
gen = w(1)*zonol{1}.generators;
for ii =2:length(zonol)
    gen = [gen w(ii)*zonol{ii}.generators];
    cen = cen + w(ii)*zonol{ii}.center;
end
cen = cen/sum(w);
gen = (1/sum(w))*gen;
Z = zonotope([cen,gen]);


    function nfro = fun(w)
        gen_inter = w(1)*zonol{1}.generators;
        cen_inter = w(1)*zonol{1}.center;
        for i =2:length(zonol)
            gen_inter = [gen_inter w(i)*zonol{i}.generators];
            cen_inter = cen_inter + w(i)*zonol{i}.center;
        end
        gen_inter = (1/sum(w))*gen_inter;
        cen_inter = cen_inter/sum(w);
        
        if strcmp(method,'volume')
            nfro = volume(zonotope([cen_inter gen_inter]));
        elseif strcmp(method,'radius')
            nfro = radius(zonotope([cen_inter gen_inter]));
        end
        
    end

end
