function ydot = EoM(y,Fc,Fe)
m = 1;
xdot = y(2);

xddot = (Fe+Fc)/m;
ydot = [y(2);xddot];

end