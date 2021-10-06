load('V03_ac.mat')
load('V03_weight.mat')
CLto = 2.2;

M0 = .773;
h = 0;

% h_sweep = linspace(0,37000,11);
M_sweep = linspace(0.05,0.4,11);

PShaft = 20e6; %watts
for i = 1:11
%     [Tout,Pout] = power2thrust_v01(M_sweep(i),ac,h_sweep(i));
    [Tout,Pout] = power2thrust_v01(M_sweep(i),ac,h);
    Ta(i) = interp1(Pout,Tout,PShaft);

end


plot(M_sweep,Ta)
% plot(Tout,Pout)
% [Tout,Pout] = power2thrust_v01(M0,ac,h)

% Pshaft = interp1(Tout,Pout,11645); % (Shaft Power)

% Pfc = Pshaft/.995/.97/.99/.99
