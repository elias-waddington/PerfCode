% HEX Drag Primer Example
% Takeoff example
% 1 HEX per stack

%% INPUTS
% Constants
gamma = 1.4;
cp = 1.006; %kj/kg-k

%Variables
% dT = 60; % deg C - why is this assumed?
% Minf = 0.25;
% h = 0;
% Hdot = 880; %KW (Qdot) - check on how to calculate
% Ar = .986; %m^2, frontal area of radiator
% Ainlet = 0.10460882; %m^2, inlet area, from Chellappa

dT = 60; % deg C - why is this assumed?
Minf = 0.773;
h = 11430;
Hdot = 870.3; %KW (Qdot) - check on how to calculate
Ar = .9858; %m^2, frontal area of radiator
Ainlet = 0.10460882; %m^2, inlet area, from Chellappa


%Environemntal (h-dervied)
[T0, a, Pinf, rhoinf] = atmosisa(h);
% T0 = 307; %deg K
vinf = Minf*a;

Tt0 = T0*(1+(gamma-1)/2*Minf^2);
dT = 396-Tt0; %423/396

%% Steps
% Step 1. Determine heat load
% defined from Chellappa's spreadsheet: 
% (now in Variables)

% Step 2. Determine HEX velocity (v1)
Mdot = Hdot/(cp*dT);

%Need to know Ar, rho 1.
rho1 = rhoinf*(1+(gamma-1)/2*Minf^2)^(1/(gamma-1)); % assume stagnation density
v1 = Mdot/Ar/rho1; 

% Step 3. Frictional losses using v1
P = 10; % can make function of Kh/Kv/sigma
dp1p2 = 1/2*rho1*v1^2*P;

% Step 4. Determine exhaust velocity v3

% v3 = sqrt((1/2*rhoinf*vinf^2-1/2*rho1*v1^2*P)/(1/2*rhoinf))
v3 = sqrt((vinf^2-P*v1^2)*(396)/Tt0); % T0 or Tt0?
%Here, do we assume the nozzle is 'ideally expanded'?

% Step 5. Determine HEX core drag
Dcore = Mdot*(vinf-v3)

%% Old & Notes
% % HEX Characteristics
% 
% 
% open questiuons about hex drag
% 
% assume a temperature increase. any way to turn this into something that is geometry-driven? (IE, inlet area & velocity to find MDot, calculate dT by that)
% I've done something, don't know if it's right

% Is the nozzle 'ideally expanded'
% how to calculate rho1


% % Using Metric, based off Chellappa's powerpoint
% 
% Vinf = a*Minf;
% 
% V1 = Vinf; % assume
% Ar = 1; % M^2
% 
% % eta2 = (Ah/Ar)^2*1/(P*r)*mu1/(rho1*V1*l);
% eta2 = 1; % Assume for now until we get HEX geometry
% 
% 
% [Eta2_rng_Kh, Kh_rng] = importKh('Kh_lookup.csv');
% [Eta2_rng_Kf, Kf_rng] = importKf('Kf_lookup.csv');
% 
% % sigma2 = (10*interp1(Eta2_rng_Kh, Kh_rng,eta2)/interp1(Eta2_rng_Kf, Kf_rng,eta2));
% 
% 
% 
% % For now, assume 10 instead of 1/sigma^2*Kf/Kh
% 
% % Te = ?
% Ti = T*(1+(gamma-1)/2*Minf^2);
% % Dcore = Hdot/Vinf*(Vinf^2*P*r^(2/3)/(cp*(Te-Ti))*10*(V1/Vinf)^2-(gamma-1)/2*Minf^2);
% 
