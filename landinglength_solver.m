function [FAR_LFL] = landinglength_solver(weight,Msweep,REsweep,CLsweep,CDout,MP_h_rng,MP_M_rng,MP_Drag_mat,MP_Wlb_mat,state,ac,CLto)

% %% Delete from here down to make into function
% FCmap_drag = allloadin_V003('FCmapR1_LD_drag.dat','n');
% FCmap_eff = allloadin_V003('FCmapR1_LD_eff.dat','n');
% load_LD1_LowDrag; 
% % run('aircraftfile_V02.m')
% run('b737aircraftfile.m')
% ac.FCsize = MP_Wlb_mat(1,1);
% 
% AR_sweep = 9.45;
% S_sweep = 1344;
% input_sweep = 2.2;
% 
% ac.wing.AR = AR_sweep(1);
% %     ac.wing.AR = AR_sweep(file_out);
% ac.wing.A = ac.wing.AR;
% % fprintf('AR = %3.1d \n',ac.wing.AR)
% ac.wing.S = S_sweep(1);
% ac.wing.bref = sqrt(ac.wing.AR*ac.wing.S);
% 
% N_sweep = 30;
% [REsweep,Msweep,CLsweep, CDout, CDoout, CDiout, CDwout, CDowing, CDofuse, Cffout] = CDgen(ac,N_sweep);
% 
% ac.size.battery_energy = 0;
% ac.size.sizing_power = 6e7;
% ac.size.max_power = 4.5e7;
% weight.fuelcell = ac.size.sizing_power/ac.FCsize;
% 
% ac.weight0 = 195000; %initial weight guess
% weight.Wpay = 35000; %payload for this mission
% ac.WF = 15000; %initial fuel guess
% state.WF0 = ac.WF;
% % [ac,weight] = weight_estimate(ac,weight);
% 
% state.W0 = 144000;
% 
% CLto = 2.2;

%% Actual functional bit

h = 0;
mission.altitude = h;
HexDrag = MP_Drag_mat(1,1)*weight.fuelcell; %assume constant drag at full power, 0.25, SL
mission.a = speedofsound(h);
mission.viscocity = airviscocity(h);
% P = airpressure(h);
mission.rho = airdensity(h);
HexDrag_coeff = HexDrag/(0.5*(speedofsound(0)*0.25)^2*mission.rho);
rho = airdensity(h);
location = zeros(100,1);
V = zeros(100,1);
ii = 1;
aoa = 5; %deg
mu = 0.4;
Vstall = sqrt(state.W0*2/(rho*ac.wing.S*CLto));

Sa = 50/tan(deg2rad(2));
Sf = (1.3*Vstall)^2/(0.2*32.2)*sin(deg2rad(2)); %1.15 or 1.3 Vstall?

%acceleration phase. DT = 1 second
% P_shaft = weight.fuelcell*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25)*FC2fan(0)/FanEff(0);
cbrk1 = 0;
V(1) = Vstall*1.15; %raymer TD estimate
while V(ii) > 0
    qbar = 0.5*V(ii)^2*mission.rho;
    HexDrag = HexDrag_coeff*qbar;
    CL = aoa*2*pi^2/180;
    L = 1/2*rho*V(ii)^2*ac.wing.S*CL;
    Re = mission.rho*V(ii)*ac.wing.MAC/mission.viscocity;
    if Re < 6e6
        Re = 6e6;
    end
    M = V(ii)/speedofsound(h);
%     [T,FPR] = thrust2powerv02(M,P_shaft*0.05);
    T = 0;
    if M < 0.11
        M = 0.11;
    end
    CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL);
    D = CD*qbar*ac.wing.S+HexDrag;
    if cbrk1 == 0
        cbrk1 = 1;
%         static_thrust = T;
%         FPR_static = FPR;
    end
    Vdot = (T-D-mu*(state.W0-L))/(state.W0/32.2);
    ii = ii +1;
    V(ii) = V(ii-1)+Vdot;
    location(ii) = location(ii-1)+V(ii);
    if ii > 250
         is_borked = 1;
         break

    end
end

Sg = max(location);

LFL = Sa+Sf+Sg;
FAR_LFL = LFL*5/3;


end