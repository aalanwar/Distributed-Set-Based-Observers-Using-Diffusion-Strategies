clear all 
close all
rand('state',123)
randn('state',223)

logname = "new_rotatingTarget.csv";

numOfNodes = 8;
%approxNumOfMeas = 1e6;
%NumOfMeas = round(approxNumOfMeas/NumOfSensor);
F= [0.992 -0.1247; 0.1247 0.992];
x(:,1) = [50;50];
h(1)=0; %[0,1]
x_noisy(:,1) = x(:,1) +5* rand(2,1);
nodeIndex(1) = 0;
dataLength= 1e4;
for i=2:dataLength
    if mod(i,12)==0%8 
        x(:,i) = F*x(:,i-1);
    else
        x(:,i) = x(:,i-1);
    end
    x_noisy(:,i) = x(:,i) +5* rand(2,1);
    
    % From 1 to numOfNodes
    nodeIndex(i) = mod(nodeIndex(i-1)+1,numOfNodes);
    if mod(i,2)==0
        h(i)=1; %[1,0]
    else
        h(i)=0; %[0,1]
    end
end
figure
plot(x_noisy(1,:),x_noisy(2,:),'r*')
hold on
plot(x(1,:),x(2,:),'*')
legend({'x noisy','x'}, 'Orientation', 'vertical', 'Location', 'NE');

time=1:dataLength;
Matrix = [ time',nodeIndex'+1,x(1,:)',x(2,:)',x_noisy(1,:)',x_noisy(2,:)',h'];
dlmwrite(strcat('logs/',logname), Matrix, 'delimiter', ',', 'precision', 20);