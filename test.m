
clear all
zonol{1} = zonotope([2 2 2;1 2 0]);
zonol{2} = zonotope([3 1 -1 1;3 1 2 0]);
zonol{3} = zonotope([1 3 -1 1;2 3 -2 0]);


%res = andAveraging1(zonol,'normGen',0.991708297763132);

%res = andAveraging1(zonol,'normGen',true,0.991708297763132);
res = andAveraging1(zonol,'volume');