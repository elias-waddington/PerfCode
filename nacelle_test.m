clear all

%inputs
load('V03_ac.mat')
ac.n.delc1 = 0.2;
ac.n.delc2 = -0.056;
M = .773;
h = 37000;
% ac.wing.bref = sqrt(ac.wing.AR*ac.wing.S);

% ONLY WORKS WITH MODDED CDgen
[REsweep, Msweep, CLsweep, CDout, CDoout, CDiout, CDwout, CDowing, CDofuse, Cffout, CDnp] = CDgen(ac,30); 



hsweep = [0	14828.26359	30461.5565	36923.81375	37000];
Mnet = [0.25	0.398	0.546	0.771	0.773];
clin = [1.154508273	0.801166242	0.77187581	0.558726316	0.557322938];

for i = 1:5
    %find Re
    M = Mnet(i);
    a = speedofsound(h); %fps
    rho = airdensity(h); %slugs/ft^3
    viscocity = airviscocity(h);
    v= M*a; %fps
    qbar = 0.5*v^2*rho;
    qbarout(i) = qbar;
    Re = rho*v*ac.wing.MAC/viscocity;


    CD = interp3(Msweep,REsweep,CLsweep,CDnp,M,Re,clin(i),'spline');
    CDgraph(i) = CD;
    
end


% plot(CL_graph,CL_graph./CDgraph)
% xlabel('C_L')
% ylabel('L/D')
% title('L/D for Cheeta')
% legend('Fuel Cell','Jet Engine','Location','southwest')