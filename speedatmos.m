function [conds] = speedatmos(alt,disa,aspeed,ispd)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% C  PROGRAM: 'speed_Atmos.f'
% C  SUB:     'speed(alt,disa,aspeed,conds,ispd)'  
% C  VERSION: 1.0
% C  AUTHOR:  Jason M Merret
% C  O. DATE: 9-12-2008
% C  M. DATE: 9-12-2008
% C  DEPT:    Preliminary Design - 0686
% C  DESC:    This subroutine/script calculates the atmospheric and speed
% C	        parameters for a get set of conditions.  It is good for 
% C		    altitude up to 65,617 ft.  Equations are drawn form the 
% C			Aerodynamic summary sheet 1 or from Barnes McCormick 
% C			"Aerodynmaics Aeronamtics and Flight Mechanics"
% C
% C  FIDs:    File ID Numbers
% C
% C  NOTES:   1479.110806 is [ao(kts @ SL ISA)*sqrt(5)]
% C
% C  INPUT:   'alt'     - Altitude (ft)
% C           'disa'    - Delta ISA in degrees celsius
% C           'aspeed'  - Speed value (Mach of KTS)
% C           'ispd'    - Speed input flag
% C                       - '1' - Mach
% C                       - '2' - KCAS
% C                       - '3' - KTAS
% C                       - '4' - KEAS
% C
% C  OUTPUT:  'conds'   -  Return value array
% C                       - conds(1)  = Altitude (ft)
% C                       - conds(2)  = Delta (pressure ratio)
% C                       - conds(3)  = Theta including DISA (temperature Ratio)
% C                       - conds(4)  = Sigma (density ratio)
% C                       - conds(5)  = Pressure (pounds/square foot)
% C                       - conds(6)  = Delta ISA (degrees C)
% C                       - conds(7)  = Temperature (Rankine)
% C                       - conds(8)  = Temperature (Fahrenheit)
% C                       - conds(9)  = Temperature (Celsius)
% C                       - conds(10) = Temperature (Kelvin)
% C                       - conds(11) = Desity (slugs/cubic feet)
% C                       - conds(12) = Speed of sound (ft/s)
% C                       - conds(13) = Dynamic Pressure (pounds/square foot)
% C                       - conds(14) = Mach Number
% C                       - conds(15) = True Airspeed (ft/s)
% C                       - conds(16) = Calibrated Airspeed (ft/s)
% C                       - conds(17) = Equivalent Airspeed (ft/s)
% C                       - conds(18) = True Airspeed (knots)
% C                       - conds(19) = Calibrated Airspeed (knots)
% C                       - conds(20) = Equivalent Airspeed (knots)
% C                       - conds(21) = Viscosity (pound-second/square foot)
% C                       - conds(22) = Reynolds Number per foot (1/ft)
% C
% C  REVISION HISTORY:
% C  REV:     DATE:           CONTACT:       DESCRIPTION:
% C
% C


tstd = 518.69;
rair = 1716.495;
pstd = 2116.22;
rstd = 0.0023769;
fpstokts = 0.592483801;
amach = aspeed;
conds = zeros(30,1);
%	Temp/Pres/Den Ratios up to 65,617 ft
if (alt<=36089.0)
    thetai = 1.0 - 0.00687535*(alt/1000.0);
    thetaf = thetai + disa*(9.0/5.0)/tstd;
    delta = thetai^5.2561;
    sigma = delta/thetaf;
end

if (alt>=36089.0)
    thetai = 1.0 - 0.00687535*(36089/1000.0);
    thetaf = thetai + disa*(9.0/5.0)/tstd;
    delta = 0.22336*exp(-.0000480634*(alt-36089.0));
    sigma = delta/thetaf;
end

%	Dimensional value calculations
tr = thetaf*tstd;
tf = tr - 459.67;
tc = (tf-32)*5.0/9.0;
tk = tc + 273.19;
a = sqrt(1.4*rair*tr);
rho = rstd*sigma;
pres = delta*pstd;

%	Speed calculations with Mach input
if (ispd==1)
    vtrue = a*amach ;
    q = 0.5*rho*vtrue*vtrue;
    qc = ((1+0.2*amach^2.0)^3.5-1.0)*pres;
    vc = (1479.11098*sqrt((1.0+qc/pstd)^(1.0/3.5)-1))/fpstokts ;
end

%	Speed calculations with KCAS input
if (ispd==2)
    ao = sqrt(1.4*rair*tstd);
    vc = amach/.592483801;
    amach = sqrt(5.0*(((1/delta)*((1+0.2*(vc/ao)^2.0)^3.5-1.0)...
        +1.0)^(1.0/3.5)-1.0));
    vtrue = a*amach;
    q = 0.5*rho*vtrue*vtrue;
end

%	Speed calculations with KTAS input
if (ispd==3)
    vtrue = amach/0.592483801;
    amach = vtrue/a;
    q = 0.5*rho*vtrue*vtrue;
    qc = ((1+0.2*amach^2.0)^3.5-1.0)*pres;
    vc = (1479.11098*sqrt((1.0+qc/pstd)^(1.0/3.5)-1))/fpstokts;
end

%	Speed calculations with KEAS input
if (ispd==4)
    vtrue = (amach/0.592483801)/sqrt(sigma);
    amach = vtrue/a;
    q = 0.5*rho*vtrue*vtrue;
    qc = ((1+0.2*amach^2.0)^3.5-1.0)*pres;
    vc = (1479.11098*sqrt((1.0+qc/pstd)^(1.0/3.5)-1))/fpstokts;
end

%	Mu and Reynolds number per foot
mu = (0.000000022697*(tr^(1.5)))/(tr+198.72);
reft = rho*vtrue/mu;

%	Speed conversions
ve = vtrue * sqrt(sigma);
ktas = vtrue*fpstokts;
kcas = vc*fpstokts;
keas = ve*fpstokts;

%	Total Air Temperature

ttotk = tk*(1.0+(1.4-1.)/2.*amach*amach);
ttotc = ttotk - 273.19;

ttotf = ttotc*9.0/5.0 +32.0;
ttotr = ttotf + 459.67;

%	Output definition
conds(1) = alt;
conds(2) = delta;
conds(3) = thetaf;
conds(4) = sigma;
conds(5) = pres;
conds(6) = disa;
conds(7) = tr;
conds(8) = tf;
conds(9) = tc;
conds(10) = tk;
conds(11) = rho;
conds(12) = a;
conds(13) = q;
conds(14) = amach;
conds(15) = vtrue;
conds(16) = vc;
conds(17) = ve;
conds(18) = ktas;
conds(19) = kcas;
conds(20) = keas;
conds(21) = mu;
conds(22) = reft;
conds(23) = ttotr;
conds(24) = ttotf;
conds(25) = ttotc;
conds(26) = ttotk;

end

