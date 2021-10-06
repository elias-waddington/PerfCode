function a = speedofsound(h)
    [T, ~, ~, ~] = atmosisa(h/3.28);
    a = 20.05*sqrt(T)*3.28084; %a in fps
end