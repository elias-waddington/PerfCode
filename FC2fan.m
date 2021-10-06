function [eff] = FC2fan(h)
%FC2fan Fuel Cell to Fan Efficiency

eff = (FanEff(h)*0.995*0.97*0.99*0.95*.98);

end

