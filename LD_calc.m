clear all

%inputs
load('V03_ac.mat')
M = .773;
h = 37000;
% ac.wing.bref = sqrt(ac.wing.AR*ac.wing.S);

[REsweep, Msweep, CLsweep, CDout, CDoout, CDiout, CDwout, CDowing, CDofuse, Cffout] = CDgen(ac,30); 

%find Re
a = speedofsound(h); %fps
rho = airdensity(h); %slugs/ft^3
viscocity = airviscocity(h);
v= M*a; %fps
qbar = 0.5*v^2*rho;
Re = rho*v*ac.wing.MAC/viscocity;

CL_sweep = linspace(0.1,.8,101);

CDgraph = zeros(1,length(CL_sweep));
CL_graph = zeros(1,length(CL_sweep));

for i = 1:length(CL_sweep)
    CL_graph(i) = CL_sweep(i);
    CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL_graph(i),'spline');
    CDgraph(i) = CD;
    
end


plot(CL_graph,CL_graph./CDgraph)
xlabel('C_L')
ylabel('L/D')
title('L/D for Cheeta')
% legend('Fuel Cell','Jet Engine','Location','southwest')