clear


AR = linspace(5,12)
[iCLAR,iCLdest] = importfig1('fig1.csv');
for i = 1:100
CLdes(i) = interp1(iCLAR,iCLdest,AR(i))
end

plot(AR,CLdes,iCLAR,iCLdest)
