N = 200;    %Number of panels
CL_des = 0.46;   %Design wing CL
sref = 2160.53; %Reference area
bref = 135.70;
cavg = sref/bref;   %Mean geometric chord

%Enter end points of wing surface
%xc1 = [0.7294 190.2256 90 90];
%xc2 = [64.442 90.5 95.6 86.422];
%yc = [0 59 59 0];
%zc = [0 4 4 0];
xc1 = [35.46429217 68.526752 64.36102 64.36102];
xc2 = [35.46429217 68.526752 74.87219 58.16147070];
yc = [0 68.21432 68.21432 0];
zc = [0 4 4 0]; %Why 4?
xcg = 60;
cp = 0.25;
rho = 0.000675954;
mu = 2.99135e-7;
nu = mu/rho;
Minf = 0.78;
sos = 968.076;
V = Minf*sos;

theta = zeros(N,1);
spacing = zeros(N,1);
semi_width = zeros(N,1);
sp = zeros(N,1);
s = zeros(N,1);
y = zeros(N,1);
z = zeros(N,1);
xle = zeros(N,1);
xte = zeros(N,1);
c = zeros(N,1);
x = zeros(N,1);
a = zeros(N,N);
abar = zeros(N+1,N+1);
b = zeros(N,1);
Re = zeros(N,1);

%Calculate reference span
bref = sref/cavg;

for i=1:N
    theta(i) = atan2(zc(2)-zc(1),yc(2)-yc(1));
    spacing(i) = sin(pi/2.*(i - 0.5)/N);
    semi_width(i) = 0.5*(sin(pi/2.*i/N)-sin(pi/2.*(i - 1.)/N));
    sp(i) = semi_width(i)*sqrt((yc(2)-yc(1))^2+(zc(2)-zc(1))^2);
    s(i) = 2.*sp(i)/bref;
    y(i) = yc(1)+spacing(i)*(yc(2)-yc(1));
    z(i) = zc(1)+spacing(i)*(zc(2)-zc(1));
    if y(i)<=25.306992  %Yehudi break <- Why defined here?
        xle(i) = xc1(1)+spacing(i)*(xc1(2)-xc1(1));
        xte(i) = xc1(4)+spacing(i)*(xc1(3)-xc1(4));
    else
        xle(i) = xc2(1)+spacing(i)*(xc2(2)-xc2(1));
        xte(i) = xc2(4)+spacing(i)*(xc2(3)-xc2(4));
    end
    c(i) = (xte(i)-xle(i));
    Re(i) = c(i)*V/nu;
    x(i) = xle(i)+0.25*c(i);
end

for i=1:N
    for j=1:N
        yp = (y(i)-y(j))*cos(theta(j))+(z(i)-z(j))*sin(theta(j));
        zp = -(y(i)-y(j))*sin(theta(j))+(z(i)-z(j))*cos(theta(j));
        r1 = zp^2+(yp-sp(j))^2;
        r2 = zp^2+(yp+sp(j))^2;

        a1 = ((yp-sp(j))/r1-(yp+sp(j))/r2)*cos(theta(i)-theta(j))+(zp/r1-zp/r2)*sin(theta(i)-theta(j));
        
        %Assuming symmetry{
        yp = (y(i)+y(j))*cos(-theta(j))+(z(i)-z(j))*sin(-theta(j));
        zp = -(y(i)+y(j))*sin(-theta(j))+(z(i)-z(j))*cos(-theta(j));
        r1 = zp^2+(yp-sp(j))^2;
        r2 = zp^2+(yp+sp(j))^2;
        a2 = ((yp-sp(j))/r1-(yp+sp(j))/r2)*cos(theta(i)+theta(j))+(zp/r1-zp/r2)*sin(theta(i)+theta(j));
        %]
        
        a(i,j) = -cavg/(4.*pi)*(a1 + a2);
    end
end

for i=1:N
    for j=1:N
        abar(i,j) = a(i,j)*s(i) + a(j,i)*s(j);
    end
    abar(i,j) = abar(i,j) + (3.15183643E-20*Re(i)^2 - 1.07237288E-11*Re(i) + 6.00306257E-03)+(0.00172387*(cavg/c(i))^2);

    b(i) = 0;
end

for i=1:N
    abar(i,N+1) = s(i)*cos(theta(i));
end
for j=1:N
    abar(N+1,j) = s(j)*cos(theta(j));
end
abar(N+1,N+1) = 0;
%Assume symmetry [
b(N+1) = CL_des/2;
%]

M = N+1;
lda = N+1;
ldb = lda;
nb = 1;

gamma=abar\b;

CL = 0;
CM = 0;
CDi = 0;
CDp = 0;
CDp2 = 0;
cn = zeros(N,1);
cdp = zeros(N,1);
cdp2 = zeros(N,1);
for i=1:N
    %Assume symmetry[
    CL = CL+2.*gamma(i)*s(i)*cos(theta(i));
    CM = CM+2.*gamma(i)*s(i)*cos(theta(i))*(xcg-(xle(i)+cp*c(i)))/cavg;
    %]
    
    cn(i) = gamma(i)*cavg/c(i);
    cdp(i) = ((3.15183643E-20*Re(i)^2 - 1.07237288E-11*Re(i) + 6.00306257E-03));%+(0.00172387*((gamma(i)*cavg/c(i)))^2));
    cdp2(i) = ((3.15183643E-20*Re(i)^2 - 1.07237288E-11*Re(i) + 6.00306257E-03)+(0.00172387*((gamma(i)*cavg/c(i)))^2));
    for j=1:N
        %Assume symmetry[
        CDi = CDi+gamma(i)*gamma(j)*s(i)*a(i,j);
        %]
    end
    CDp = CDp+((3.15183643E-20*Re(i)^2 - 1.07237288E-11*Re(i) + 6.00306257E-03))*s(i)*2;%+(0.00172387*((gamma(i)*cavg/c(i)))^2))*c(i)/(sref/2);
    CDp2 = CDp2+((3.15183643E-20*Re(i)^2 - 1.07237288E-11*Re(i) + 6.00306257E-03)+(0.00172387*((gamma(i)*cavg/c(i)))^2))*s(i)*2;
end

ar = bref^2/sref;
e = CL^2/(pi*ar*CDi);

% Add graph step
plot(y,cn)
xlabel('Half-span, ft')
ylabel('cl')
title('Spanwise Lift Distribution')