%iterative ML/D vs M at Constant W plot
%constant weight, variable M, CD
clc
clear
close all

run('ESDU_bwbaircraftfile.m')
M = linspace(.25,.95);
% M = [0.3,.9];
% M = linspace(0.65,0.71,5);
% M = [0.67,0.7]
mission.altitude = 35000; %ft


mission.a = speedofsound(mission.altitude);
mission.rho = airdensity(mission.altitude); %slugs/ft^3
mission.viscocity = airviscocity(mission.altitude);
W = ac.weight0*.95; %lbs

%init
CDgraph = zeros(1,length(M));
CLgraph = zeros(1,length(M));
CDograph = zeros(1,length(M));
CDigraph = zeros(1,length(M));
CDwgraph = zeros(1,length(M));
LDgraph = zeros(1,length(M));
MLDgraph = zeros(1,length(M));
CDCgraph = zeros(1,length(M));
CDPgraph = zeros(1,length(M));
TRgraph = zeros(1,length(M));

for i = 1:length(M)
    mission.M = M(i);
    mission.v_cruise = mission.M*mission.a; %in fps
    mission.qbar = 0.5*mission.v_cruise^2*mission.rho;
    
    [CD,CL,CDo,CDi,CDw,CDC,CDP] = dragBWB(ac,mission,W);
    Tr = 1/2*mission.rho*mission.v_cruise^2*ac.wing.S*CD;
    CDgraph(i) = CD;
    CLgraph(i) = CL;
    CDograph(i) = CDo;
    CDigraph(i) = CDi;
    CDwgraph(i) = CDw;
    LDgraph(i) = CL/CD;
    MLDgraph(i) = LDgraph(i)*M(i);
    CDCgraph(i) = CDC;
    CDPgraph(i) = CDP;
    TRgraph(i) = Tr;
    PRgraph(i) = Tr*mission.v_cruise;
%     MLD(i) = CLCD(i)*M(i);
    
    
end
% 
% figure(1)
% plot(M,MLDgraph)
% title('BWB ML/D at Constant Weight')
% xlabel('Mach Number')
% ylabel('ML/D')
% 
% bestcruise = (find(max(MLDgraph)==MLDgraph));
% bestmessage = ' Best cruise M is %4.3f \n Best cruise L/D is %4.3f \n Best cruise ML/D is %4.3f \n';
% fprintf(bestmessage,M(bestcruise),LDgraph(bestcruise),MLDgraph(bestcruise))
% 
% figure(2)
% plot(M,LDgraph)
% xlabel('Mach Number')
% ylabel('L/D')
% title('BWB L/D at Constant Weight')
% 
% figure(3)
% plot(CDgraph,CLgraph)
% xlabel('CL')
% ylabel('CD')
% title('BWB Drag Polar, variable mach')
% 
% figure(4)
% plot(M,CDwgraph)
% xlabel('Mach Number')
% ylabel('Wave Drag, CD')
% title('BWB Wave Drag, variable Mach')

figure(5)
plot(M,TRgraph)
xlabel('Mach No.')
ylabel('Thrust Required')
title('BWB TR, variable Mach')

figure(6)
plot(M,PRgraph)
xlabel('Mach No.')
ylabel('Power Required')
title('BWB PR, variable Mach')