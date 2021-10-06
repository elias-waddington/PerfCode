function [TOFL,static_thrust,FPR_static] = takeoff_solver(weight,Msweep,REsweep,CLsweep,CDout,MP_h_rng,MP_M_rng,MP_Drag_mat,MP_Wlb_mat,state,ac,CLto)

h = 0;
mission.altitude = h;
HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,0.25)*weight.fuelcell; %assume constant drag at full power, 0.25, SL
mission.a = speedofsound(h);
mission.viscocity = airviscocity(h);
P = airpressure(h);
mission.rho = airdensity(h);
HexDrag_coeff = HexDrag/(0.5*(speedofsound(0)*0.25)^2*mission.rho);
rho = airdensity(h);
location = zeros(100,1);
V = zeros(100,1);
ii = 1;
aoa = 5; %deg
mu = 0.05;
Vstall = sqrt(state.W0*2/(rho*ac.wing.S*CLto));
Vrotate = Vstall*1.08;
%acceleration phase. DT = 1 second
P_shaft = weight.fuelcell*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25)*FC2fan(0)/FanEff(0);
cbrk1 = 0;
while V(ii) < Vrotate
    qbar = 0.5*V(ii)^2*mission.rho;
    HexDrag = HexDrag_coeff*qbar;
    CL = aoa*2*pi^2/180;
    L = 1/2*rho*V(ii)^2*ac.wing.S*CL;
    Re = mission.rho*V(ii)*ac.wing.MAC/mission.viscocity;
    if Re < 6e6
        Re = 6e6;
    end
    M = V(ii)/speedofsound(h);
    [T,FPR] = thrust2powerv02(M,P_shaft,ac);
    if M < 0.11
        M = 0.11;
    end
    CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL,'spline');
    D = CD*qbar*ac.wing.S+HexDrag;
    if cbrk1 == 0
        cbrk1 = 1;
        static_thrust = T;
        FPR_static = FPR;
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
rotate_begin = ii;
%Normal Take-off

gamma = asind((T-D-HexDrag)/state.W0);
if gamma > 15
    gamma = 15;
end
R = Vrotate^2/(0.1*32.2);
htr = R*(1-cosd(gamma));
if htr < 35
Str = R*sind(gamma);
Sc = (35-htr)/tand(gamma);
elseif htr > 35
Str = sqrt(R^2-(R-35)^2);
Sc = 0;
end

 S_AETO = Sc + Str + location(rotate_begin);

%OEI TO case
gamma = asind((T*2/3-D-HexDrag)/state.W0);
if gamma > 9
    gamma = 9;
end
R = Vrotate^2/(0.1*32.2);
htr = R*(1-cosd(gamma));
if htr < 35
Str = R*sind(gamma);
Sc = (35-htr)/tand(gamma);
elseif htr > 35
Str = sqrt(R^2-(R-35)^2);
Sc = 0;
end

S_OEITO = Sc + Str + location(rotate_begin);

%OEI Stop Case
mu = 0.05;
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
    [T,FPR(ii)] = thrust2powerv02(M,P_shaft,ac);
    if M < 0.11
        M = 0.11;
    end
    CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL,'spline');
    D = CD*qbar*ac.wing.S+HexDrag;
    T = max(0,T - T*(ii - rotate_begin)/5); %lbs, fix this later
    mu = min(0.5,mu+(ii - rotate_begin)*0.25);
    Vdot = (T*2/3-D-mu*(state.W0-L))/(state.W0/32.2);
    ii = ii +1;
    V(ii) = V(ii-1)+Vdot;
    location(ii) = location(ii-1)+V(ii);
end
ii = rotate_begin;
S_OEIstop = max(location);

%All Engine Stop case
mu = 0.05;
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
    [T,FPR(ii)] = thrust2powerv02(M,P_shaft,ac);
    if M < 0.11
        M = 0.11;
    end
    CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL,'spline');
    D = CD*qbar*ac.wing.S+HexDrag;
    T = max(0,T - T*(ii - (rotate_begin+1))/5); %lbs, fix this later
    mu = min(0.5,mu+(ii - (rotate_begin+1))*0.25);
    Vdot = (T-D-mu*(state.W0-L))/(state.W0/32.2);
    ii = ii +1;
    V(ii) = V(ii-1)+Vdot;
    location(ii) = location(ii-1)+V(ii);
end
% rotate_begin = ii;
S_AEstop = max(location);
% S_AEstop = 0;
S_vector = [S_AETO, S_OEITO, S_OEIstop, S_AEstop];
TOFL = max(S_vector);

end