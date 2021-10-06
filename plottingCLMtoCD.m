%iterative ML/D vs M at Constant W plot
%constant weight, variable M, CD
clc
clear
close all
% run('ESDU_bwbaircraftfile.m')
run('aircraftfile_V02.m')
M = linspace(0.3,.9);

% M = [0.7,0.71]
mission.altitude = 37000; %ft
CLin = 0.2;
CLout = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8];

%init
CDgraph = zeros(length(CLout),length(M));
CLgraph = zeros(length(CLout),length(M));
CDograph = zeros(length(CLout),length(M));
CDigraph = zeros(length(CLout),length(M));
CDwgraph = zeros(length(CLout),length(M));
LDgraph = zeros(length(CLout),length(M));
MLDgraph = zeros(length(CLout),length(M));
CDCgraph = zeros(length(CLout),length(M));
CDPgraph = zeros(length(CLout),length(M));
mission.a = speedofsound(mission.altitude);
mission.rho = airdensity(mission.altitude); %slugs/ft^3
mission.viscocity = airviscocity(mission.altitude);
k = 1;

for j = 1:length(CLout)
    CLin = CLout(j)
    for i = 1:length(M)
        mission.M = M(i);
        mission.v_cruise = mission.M*mission.a; %in fps
        mission.qbar = 0.5*mission.v_cruise^2*mission.rho;
        W = mission.qbar.*CLin*ac.wing.S;

        [ac,CD,CL,CDo,CDi,CDw,CDC,CDP] = dragBWB(ac,mission,W);
        CDgraph(j,i) = CD;
        CLgraph(j,i) = CL;
        CDograph(j,i) = CDo;
        CDigraph(j,i) = CDi;
        CDwgraph(j,i) = CDw;
    %     PDgraph(i) = ProfileDrag;
        LDgraph(j,i) = CL/(CD);
        MLDgraph(j,i) = LDgraph(i)*M(i);    
        CDCgraph(j,i) = CDC;
        CDPgraph(j,i) = CDP;
    %     MLD(i) = CLCD(i)*M(i);
%         CLgraph(k) = CLin;
%         CDgraph(k) = CD;
%         CDograph(k) = CDo;
%         CDigraph(k) = CDi;
%         CDwgraph(k) = CDw;
%         CDCgraph(k) = CDC;
%         CDPgraph(k) = CDP;
%         Mgraph(k) = M(i);
        
        k = k+1;


    end
end


% outputtable = table(CLgraph',Mgraph',CDgraph',CDograph',CDigraph',CDwgraph',CDCgraph',CDPgraph');
% outputtable.Properties.VariableNames = {'CL' 'M' 'Total Aircraft CD' 'Profile Drag' 'Induced Drag' 'Wave Drag' 'Compressible Drag' 'Pressure Drag'};
% writetable(outputtable,'CLMoutputs.csv');





figure(1)
plot(M,MLDgraph)
title(sprintf('ML/D at Constant CL %f',CLin))
xlabel('Mach Number')
ylabel('ML/D')
legend('0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1')

% bestcruise = (find(max(MLDgraph)==MLDgraph));
% bestmessage = ' Best cruise M is %4.3f \n Best cruise L/D is %4.3f \n Best cruise ML/D is %4.3f \n';
% fprintf(bestmessage,M(bestcruise),LDgraph(bestcruise),MLDgraph(bestcruise))

figure(2)
plot(M,LDgraph)
xlabel('Mach Number')
ylabel('L/D')
title(sprintf('L/D at Constant CL %f',CLin))
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8')

figure(3)
plot(M,CDgraph)
xlabel('M')
ylabel('CD')
title('Drag Polar, variable mach')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8')

figure(4)
plot(M,CDwgraph)
xlabel('Mach Number')
ylabel('Wave Drag, CD')
title('Wave Drag, variable Mach')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8')

figure(5)
plot(M,CDCgraph)
xlabel('Mach Number')
ylabel('Wave Drag')
title('Wave Drag, variable Mach, Just Compressible')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8')

figure(6)
plot(M,CDPgraph)
xlabel('Mach Number')
ylabel('Wave Drag')
title('Wave Drag, variable Mach, Just Pressure')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8')