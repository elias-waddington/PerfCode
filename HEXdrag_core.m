function [Dcore] = HEXdrag_core(Minf,h,Ar,Hdot)
% Constants
gamma = 1.4;
cp = 1006; %kj/kg-k

%Environemntal (h-dervied)
[T0, a, Pinf, rhoinf] = atmosisa(h/3.2808399);
vinf = Minf*a;
Tt0 = T0*(1+(gamma-1)/2*Minf^2);
dT = 396-Tt0; %423 or 396, assume constant, not great

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
P = 10; % can make function of Kh/Kv/sigma, for now, assume Drela's  typical value
% dp1p2 = 1/2*rho1*v1^2*P;

% Step 4. Determine exhaust velocity v3
v3 = sqrt((vinf^2-P*v1^2)*(396)/Tt0); % T0 or Tt0?

% Step 5. Determine HEX core drag
Dcore = Mdot*(vinf-v3); % in Newtons
Dcore = Dcore* 0.22480894;

end

