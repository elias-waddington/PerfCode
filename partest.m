% Function to test how PARFOR works
clear all

% x = linspace(0,1,1e6);
tic
n = 200;
A = 500;
a = zeros(1,n);
parfor i = 1:n
    a(i) = max(abs(eig(rand(A))));
end
toc