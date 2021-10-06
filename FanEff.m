function prop_eff = FanEff(h)
% if h < 35000
%     prop_eff = 0.7 + 0.15*h/35000;
% else
%     prop_eff = 0.85;
% end

if h > 37000
    h = 37000;
end

alt = [-5,25600,33600,37005];
eff = [0.481,0.663,0.727,0.756];

prop_eff = interp1(alt,eff,h);


end