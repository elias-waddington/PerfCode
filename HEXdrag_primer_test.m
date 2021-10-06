% HEX Drag Primer Example


Minf = 0.773;
gamma = 1.4;
h = 37000/3.3;
[T, a, P, rho] = atmosisa(h);
cp = 1; %kj/kg-k

% HEX Characteristics


% Using Metric for ease of calculation

Vinf = a*Minf;

V1 = Vinf; % assume
Ar = 1; % M^2
Hdot = 1000; %??? Heat load

% eta2 = (Ah/Ar)^2*1/(P*r)*mu1/(rho1*V1*l);
eta2 = 1; % Assume for now until we get HEX geometry


[Eta2_rng_Kh, Kh_rng] = importKh('Kh_lookup.csv');
[Eta2_rng_Kf, Kf_rng] = importKf('Kf_lookup.csv');

% sigma2 = (10*interp1(Eta2_rng_Kh, Kh_rng,eta2)/interp1(Eta2_rng_Kf, Kf_rng,eta2));



% For now, assume 10 instead of 1/sigma^2*Kf/Kh

% Te = ?
Ti = T*(1+(gamma-1)/2*Minf^2);
Dcore = Hdot/Vinf*(Vinf^2*P*r^(2/3)/(cp*(Te-Ti))*10*(V1/Vinf)^2-(gamma-1)/2*Minf^2);

