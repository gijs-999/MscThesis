x0 = 1;
xdot0 = 0;

n = 1000;
T = 1;
ref.t = (0:n)/(n)*T ;
ref.x = 0*ref.t;
ref.xdot = 0*ref.t;

y0 = [x0;xdot0];

y = ode15s(@(t,y) ode(y,t,ref),[0,T],y0)

figure(1); clf
subplot(2,1,1)
plot(y.x,y.y(1,:))
subplot(2,1,2)
plot(y.x,y.y(2,:))


function ydot = ode(y,t,ref)
m = 1;
xdot = y(2);

F = force(y,t,ref);

xddot = F/m;
ydot = [xdot;xddot];

end

function F = force(y,t,ref)
ke = 10^6;
be = 10;
kc = 4000;
bc = 80;

x = y(1);
xdot = y(2);

xref = interp1(ref.t,ref.x,t);
xdotref = interp1(ref.t,ref.xdot,t);

if x>0
    Fe = 0;
else
    Fe = -ke*x+be*xdot;
end
    
Fc = kc*(xref-x)+bc*(xdotref-xdot);

F = Fe + Fc;
end


