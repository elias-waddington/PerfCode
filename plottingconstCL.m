%iterative ML/D vs M at Constant W plot
%constant weight, variable M, CD
clc
clear
close all
% run('ESDU_bwbaircraftfile.m')
run('aircraftfile_V01.m')
M = linspace(0.3,.9);
% M = .786;

% M = [0.7,0.71]
mission.altitude = 35000; %ft
CLin = 0.3;
CLout = [0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1];

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
mission.a = speedofsound(mission.altitude);
mission.rho = airdensity(mission.altitude); %slugs/ft^3
mission.viscocity = airviscocity(mission.altitude);

for i = 1:length(M)
    mission.M = M(i);
    mission.v_cruise = mission.M*mission.a; %in fps
    mission.qbar = 0.5*mission.v_cruise^2*mission.rho;
    W = mission.qbar.*CLin*ac.wing.S;
    
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
title(sprintf('ML/D at Constant CL %f',CLin))
xlabel('Mach Number')
ylabel('ML/D')

bestcruise = (find(max(MLDgraph)==MLDgraph));
bestmessage = ' Best cruise M is %4.3f \n Best cruise L/D is %4.3f \n Best cruise ML/D is %4.3f \n';
fprintf(bestmessage,M(bestcruise),LDgraph(bestcruise),MLDgraph(bestcruise))

figure(2)
plot(M,LDgraph)
xlabel('Mach Number')
ylabel('L/D')
title(sprintf('L/D at Constant CL %f',CLin))

figure(3)
plot(M,CDgraph)
xlabel('M')
ylabel('CD')
title('Drag Polar, variable mach')

figure(4)
plot(M,CDwgraph)
xlabel('Mach Number')
ylabel('Wave Drag, CD')
title('Wave Drag, variable Mach')

figure(5)
plot(M,CDCgraph)
xlabel('Mach Number')
ylabel('Wave Drag')
title('Wave Drag, variable Mach, Just Compressible')

figure(6)
plot(M,CDPgraph)
xlabel('Mach Number')
ylabel('Wave Drag')
title('Wave Drag, variable Mach, Just Pressure')