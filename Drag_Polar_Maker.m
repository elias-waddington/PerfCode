clear all

% Drag Polar Generation Code

% inputs
load('V03_ac.mat')
M = .773;
h = 37000;

%find Re
a = speedofsound(h); %fps
rho = airdensity(h); %slugs/ft^3
viscocity = airviscocity(h);
v= M*a; %fps
qbar = 0.5*v^2*rho;
Re = rho*v*ac.wing.MAC/viscocity;

%calculate CD map
[REsweep,Msweep,CLsweep, CDout, CDoout, CDiout, CDwout, CDowing, CDofuse, Cffout] = CDgen(ac,30);

%init
CL_sweep = linspace(0.05,0.6,101);
CD_graph = zeros(1,length(CL_sweep));


for i = 1:length(CL_sweep)
    
    CD_graph(i) = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL_sweep(i),'spline');
    
end

plot(CD_graph,CL_sweep)