function [Pshaft,TD] = taxi_estimator(weight,Msweep,REsweep,CLsweep,CDout,FCmap_drag,MP_M_rng,MP_Drag_mat,MP_Wlb_mat,state,ac,CLto)
%% Actual actual function bit

%assume taxi v
V = 25; %fps
h = 0;
mission.rho = airdensity(h);
mission.viscocity = airviscocity(h);
aoa = 5;
mu = 0.05;

%can calculate aero drag
qbar = 0.5*V^2*mission.rho;
CL = aoa*2*pi^2/180;
L = 1/2*mission.rho*V^2*ac.wing.S*CL;
Re = mission.rho*V*ac.wing.MAC/mission.viscocity;
if Re < 6e6
    Re = 6e6;
end
M = V/speedofsound(h);
[Tout,Pout] = power2thrust_v01(M,ac,h);
if M < 0.25
    M = 0.25;
end
CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL,'spline');

D = CD*qbar*ac.wing.S;
TD = D+mu*(state.W0-L);
TD2 = TD;
err = 100;
while err > 2
    TD3 = TD2;
    Pshaft = interp1(Tout,Pout,TD2);
    HexDrag = interp3D_V003(FCmap_drag,1,Pshaft/FC2shaft(h)/weight.fuelcell,M,h,'n')*weight.fuelcell;
    if HexDrag > 100000
        HexDrag = 100;
    end
    TD2 = TD+HexDrag;
    err = abs(TD3-TD2);
end


end