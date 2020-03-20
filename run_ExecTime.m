clear all

zonol{1} = zonotope([10 2 2;1 2 0]);
zonol{2} = zonotope([3 1 -1 1;3 1 2 0]);
%zonol{3} = zonotope([1 3 -1 1;2 3 -2 0]);
res = andAveraging1(zonol);
figure; hold on
plot(zonol{1},[1,2],'r');
plot(zonol{2},[1,2],'r-+');
%plot(zonol{3},[1,2],'r-*');
plot(res,[1,2],'k');

%%diffusion update
loops = 100000;
% % % for i=1:loops
% % %     zonol{1}= zonotope.generateRandom(2,[2;1],20);
% % %     zonol{2}= zonotope.generateRandom(2,[3;3],20);
% % %     zonol{3}= zonotope.generateRandom(2,[1;2],20);
% % %     zonol{4}= zonotope.generateRandom(2,[1.5;2],20); 
% % %     zonol{5}= zonotope.generateRandom(2,[2;2],20);
% % %     zonol{6}= zonotope.generateRandom(2,[2.1;2.4],20); 
% % %     tic;
% % %     res = andAveraging1(zonol);
% % %     t=toc;
% % %     execTime(i) = t;
% % % end
% % % mean(execTime)
% figure
% hold on
% plot(zonol{1},[1,2],'r');
% plot(zonol{2},[1,2],'r-+');
% plot(zonol{3},[1,2],'r-*');
% plot(res,[1,2],'k');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% hl{1} = [1 0];
% Rl{1} = 5;
% yl{1} = -2;
% 
% hl{2} = [0 1];
% Rl{2} = 3;
% yl{2} = 2;
% 
% hl{3} = [1 1];
% Rl{3} = 3;
% yl{3} = 2;
% 
% hl{4} = [1.5 0];
% Rl{4} = 5.4;
% yl{4} = -2.3;
% % 
% hl{5} = [0 1.5];
% Rl{5} = 3.4;
% yl{5} = 2.3;
% 
% hl{6} = [1.5 1.5];
% Rl{6} = 3.4;
% yl{6} = 2.3;
% %z = zonotope([1 2 2 2 6 2 8;1 2 2 0 5 0 6 ]);
% for i=1:loops
%     z= zonotope.generateRandom(2,[1;2],20);
%      tic;
%     res_zono= intersectZonoStrip(z,hl,Rl,yl);
%     t=toc;
%     execTime(i) = t;   
% end
% mean(execTime)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% F= [0.992 -0.1247; 0.1247 0.992];
% Q = diag([0.5 0.5]);
% zQ = zonotope([[0;0],Q]);
% for i=1:loops
%     z= zonotope.generateRandom(2,[2;1],20);
%     tic;
%     Fz = F*z;
%     newzono = zQ + Fz;
%     t=toc;
%     execTime(i) = t;
% end
%  mean(execTime)
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  hl{1} = [1 0];
% Rl{1} = 5;
% yl{1} = -2;
% 
% hl{2} = [0 1];
% Rl{2} = 3;
% yl{2} = 2;
% 
% hl{3} = [1 1];
% Rl{3} = 3;
% yl{3} = 2;
% 
% hl{4} = [1.5 0];
% Rl{4} = 5.4;
% yl{4} = -2.3;
% % % 
% hl{5} = [0 1.5];
% Rl{5} = 3.4;
% yl{5} = 2.3;
% 
% hl{6} = [1.5 1.5];
% Rl{6} = 3.4;
% yl{6} = 2.3;
% %z = zonotope([1 2 2 2 6 2 8;1 2 2 0 5 0 6 ]);
% F= [0.992 -0.1247; 0.1247 0.992];
% for i=1:loops
%     z= zonotope.generateRandom(2,[1;2],20);
%      tic;
%     res_zono= luenberger(z,hl,Rl,yl,F);
%     t=toc;
%     execTime(i) = t;   
% end
% mean(execTime)
%poly = mptPolytope([1 0;-1 0; 0 1;0 -1; 1 1;-1 -1],[3;7;5;1;5;1]);