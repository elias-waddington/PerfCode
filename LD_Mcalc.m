clear all

%inputs
load('V03_ac.mat')
CL_in = .4;
h = 37000;
% ac.wing.bref = sqrt(ac.wing.AR*ac.wing.S);

[REsweep,Msweep,CLsweep, CDout, CDoout, CDiout, CDwout, CDowing, CDofuse, Cffout] = CDgen(ac,30); 

%find Re
a = speedofsound(h); %fps
rho = airdensity(h); %slugs/ft^3
viscocity = airviscocity(h);
% v= M*a; %fps
% qbar = 0.5*v^2*rho;
% Re = rho*v*ac.wing.MAC/viscocity;

CL_sweep = linspace(0.1,.8,101);

CDgraph = zeros(1,length(CL_sweep));
CL_graph = ones(1,length(CL_sweep))*CL_in;
M_sweep = linspace(0.1,0.8,101);

for i = 1:length(M_sweep)
    CL_graph(i) = CL_sweep(i);
    M = M_sweep(i);
    v = M*a; %fps
    Re = rho*v*ac.wing.MAC/viscocity;
    CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL_sweep(i),'spline');
    CDgraph(i) = CD;
    
end


plot(CL_graph,CL_graph./CDgraph)
xlabel('Mach')
ylabel('ML/D')
title('ML/D for Cheeta')
% legend('Fuel Cell','Jet Engine','Location','southwest')