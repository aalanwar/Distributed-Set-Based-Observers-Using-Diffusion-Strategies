function [ dislist ] = zonoDiff( zonolist )
%ZONODIFF Summary of this function goes here
%   Detailed explanation goes here
dislist = [];
for i=1:length(zonolist)
    for j=i+1:length(zonolist)
        dislist = [dislist ;hdis(zonolist{i},zonolist{j})];
    end
end
% dismean = mean(dislist);
% disstd = std(dislist);
% dismax = max(dislist);

     function maxdis = hdis(z1,z2)
         vz1 = z1.vertices;
         vz2 = z2.vertices;
         vz1mat = vz1(:,:);
         vz2mat = vz2(:,:);
         maxdis= HausdorffDist(vz1mat',vz2mat');
     end
end

