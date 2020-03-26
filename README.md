# Distributed-Set-Based-Observers-Using-Diffusion-Strategy

This repo is the code and data used to evaulate our [paper](https://arxiv.org/abs/2003.10347)   <br />
Amr Alanwar, Jagat Jyoti Rath, Hazem Said, and Matthias Althoff "Distributed-Set-Based-Observers-Using-Diffusion-Strategy"<br />
<br />
<br />
This repo to generate the following set-based estimation video. 
[![video](https://img.youtube.com/vi/ioKRCaVxyOQ/0.jpg)](https://youtu.be/ioKRCaVxyOQ)

<br />
We propose two distributed set-based observers using <br />
1- Set-membership approach <br />
2- Interval-based approach <br />
 for a linear discrete-time dynamical system with bounded modeling and measurement uncertainties. <br />
<br />
<br />
The Set-membership approach consists of three steps <br />
a- Measurement update: It consists of overapproximating the intersection between a strips and zonotope which is represented using the following <br /> <br />
<p align="center">
<img
src="output/meas2.png"
raw=true
alt="Subject Pronouns"
width=500
/>
</p>
The main function to intersect strips and a zonotope is "intersectZonoStrip1.m" under the "utilities" folder.<br />
<br />
b- Diffusion update: It consists of overapproximating the intersection between a multiple zonotopes by a zonotope which is represented using the following <br /><br />
<p align="center">
<img
src="output/diff2.png"
raw=true
alt="Subject Pronouns"
width=500
/>
</p><br />
The main function to intersect zonotopes is "andAveraging1.m" under the "utilities" folder
<br />
c- Time update
<br />
<br />
<br />
<br />
To run the distribted set-based observer on the rotatingTarget.csv file, do the following
1- Download [CORA](https://github.com/TUMcps/CORA) and [MPT](https://www.mpt3.org/) toolboxs.
2- Add CORA nad MPT folder and subfolders to the Matlab path.  
3- cd to the repo folder.
4- Open the main file "run_rotatingTarget" <br /> 
5- choose which algorithm to be executed at line ~150 algorithm = 'set-membership' or algorithm = 'interval-based'
6- To save the movie, set<br />
SAVEMOVIE = true; 
at to save the generated movie under the video folder.<br />
7- This generates mat file (temp by default) under the cache folder. <br />
8- run "plot_rotatingTarget_state.m" which choosing loading the cache temp at the beggining load('cache/temp'). This plots a figure close to the following figures <br /> <br />
<p align="center">
<img
src="output/state_x1.png"
raw=true
alt="Subject Pronouns"
width=500
/>
</p><br />
<p align="center">
<img
src="output/state_x2.png"
raw=true
alt="Subject Pronouns"
width=500
/>
</p><br />
9- To change network connectivity edit the network list ~line 120 in NetworkManager_R.m
%2 neig
 network= { [8 1 2],[1 2 3],[2 3 4],[3 4 5],[4 5 6],[5 6 7],[6 7 8],[7 8 1]}; 
<br />
10- To enable/disable the diffusion step set the following to 1/0. <br />
diffEnable =1;<br />
<br />
11- To regenerate the measurement file, edit and run measGenerator.m. rename new_rotatingTarget to rotatingTarget if you are sure about using the new file.
<br />
