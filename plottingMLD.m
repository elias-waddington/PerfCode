%iterative ML/D vs M at Constant W plot
%constant weight, variable M, CD
clc
clear
close all
% run('ESDU_bwbaircraftfile.m')
% run('aircraftfile_V02.m')
run('b737aircraftfile.m')
M = linspace(0.3,.9);
% M = [0.7,0.71]
mission.altitude = 37000; %ft


mission.a = speedofsound(mission.altitude);
mission.rho = airdensity(mission.altitude); %slugs/ft^3
mission.viscocity = airviscocity(mission.altitude);
% W = ac.weight0*.95; %lbs
W = 200000;
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

for i = 1:length(M)
    mission.M = M(i);
    mission.v_cruise = mission.M*mission.a; %in fps
    mission.qbar = 0.5*mission.v_cruise^2*mission.rho;
    
    [ac,CD,CL,CDo,CDi,CDw,CDC,CDP] = dragBWB(ac,mission,W);
    CDgraph(i) = CD;
    CLgraph(i) = CL;
    CDograph(i) = CDo;
    CDigraph(i) = CDi;
    CDwgraph(i) = CDw;
%     PDgraph(i) = ProfileDrag;
    LDgraph(i) = CL/(CD);
    MLDgraph(i) = LDgraph(i)*M(i);    
    CDCgraph(i) = CDC;
    CDPgraph(i) = CDP;
%     MLD(i) = CLCD(i)*M(i);
    
end

figure(1)
plot(M,MLDgraph)
title('ML/D at Constant Weight')
xlabel('Mach Number')
ylabel('ML/D')
legend('CHEETA','location','SouthEast')

bestcruise = (find(max(MLDgraph)==MLDgraph));
bestmessage = ' Best cruise M is %4.3f \n Best cruise L/D is %4.3f \n Best cruise ML/D is %4.3f \n';
fprintf(bestmessage,M(bestcruise),LDgraph(bestcruise),MLDgraph(bestcruise))

figure(2)
plot(M,LDgraph)
xlabel('Mach Number')
ylabel('L/D')
title('L/D at Constant Weight')

figure(3)
plot(CDgraph,CLgraph)
xlabel('CD')
ylabel('CL')
title('Drag Polar, variable mach')

figure(4)
plot(M,CDwgraph)
xlabel('Mach Number')
ylabel('Wave Drag, CD')
title('BWB Wave Drag, variable Mach')