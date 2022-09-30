clear all
%% generate demonstration reference
x0 = 0.35;
xdot0 = 0;

T = 2;
n = T*1000;
ref.t = (0:n-1)/(n-1)*T ;

ref.uddot = [-1.25*ones(0.2*n,1);
            zeros(0.2*n,1);
             1.25*ones(0.2*n,1);
             zeros(0.4*n,1)];
ref.udot = xdot0+cumtrapz(ref.t,ref.uddot);
ref.u = x0+cumtrapz(ref.t,ref.udot);

figure(102); clf; hold on;

plot(ref.t,ref.u);
plot(ref.t,ref.udot);
plot(ref.t,ref.uddot);

%% perform demonstration and save learning reference
y0 = [x0;xdot0];
controller = @(y,t,ref) controller_impedance(y,t,ref);
y = ode15s(@(t,y) ode(y,t,ref,controller),[0,T],y0);

for i = 1:length(y.y)
[F(i),Fe(i),Fc(i)] = force(y.y(:,i),y.x(i),ref, controller);
end

ref.Fe = interp1(y.x,Fe,ref.t);
ref.Fc = interp1(y.x,Fc,ref.t);
% first order filter on Fe
kf = 10;
options = odeset("MaxStep",1e-2);
y2 = ode15s(@(t,Fe_) kf*(interp1(ref.t,ref.Fe,t) - Fe_),[0,T],Fe(1),options) ;

ref.x = interp1(y.x,y.y(1,:),ref.t);
ref.xdot = interp1(y.x,y.y(2,:),ref.t);
ref.Fe_ = interp1(y2.x,y2.y,ref.t);

% ref.Fe_ = ref.Fe;

% plot: demonstration
figure(1); clf; 
subplot(4,1,1);hold on
plot(y.x,y.y(1,:))
plot(ref.t,ref.u,'k--')
ylabel("$x$ [m]")
grid on; box on;

subplot(4,1,2); hold on
plot(y.x,y.y(2,:))
plot(ref.t,ref.udot,'k--')
ylabel("$\dot{x}$ [m/s]")
grid on; box on;

subplot(4,1,3); hold on
% plot(y.x,Fe)
plot(ref.t,ref.Fe)
plot(ref.t,ref.Fe_)
ylabel("$F_e$ [N]")
grid on; box on;

subplot(4,1,4); hold on
plot(y.x,Fc)
ylabel("$F_c$ [N]")
grid on; box on;

%% perform learned reference with pos/force (no filter)

controller = @(y,t,ref) controller_posforce(y,t,ref);
y_f = ode15s(@(t,y) ode(y,t,ref, controller),[0,T],y0);

for i = 1:length(y_f.y)
[F_p(i),Fe_p(i),Fc_p(i)] = force(y_f.y(:,i),y_f.x(i),ref,controller);
end

% plot: pos/force
figure(2); clf; 
subplot(4,1,1);hold on
plot(ref.t,ref.u,'k--')
plot(ref.t,ref.x)
plot(y_f.x,y_f.y(1,:))
ylabel("$x$ [m]")
grid on; box on;
legend("$u$","$x^d=x^i$","$x^p$")

subplot(4,1,2); hold on
plot(ref.t,ref.udot,'k--')
plot(ref.t,ref.xdot)
plot(y_f.x,y_f.y(2,:))
ylabel("$\dot{x}$ [m/s]")
grid on; box on;
legend("$\dot{u}$","$\dot{x}^d=\dot{x}^i$","$\dot{x}^p$")

subplot(4,1,3); hold on
plot(y.x,Fe)
plot(y_f.x,Fe_p)
% plot(ref.t,ref.Fe_,"k--")
ylabel("$F_e$ [N]")
grid on; box on;
legend("$F_e^d=F_e^i$","$F_e^f$","$\tilde{F}_e^d$")

subplot(4,1,4); hold on
plot(ref.t,ref.Fc)
plot(y_f.x,Fc_p)
ylabel("$F_c$ [N]")
grid on; box on;
legend("$F_c^d=F_c^i$","$F_c^f$","$\tilde{F}_e^d$")

%% plot for filtered force reference
figure(202); clf
subplot(4,1,3); hold on
plot(y.x,Fe)
plot(ref.t,ref.Fe_)
ylabel("$F_e$ [N]")
legend("$F_e^d$","$\tilde{F}_e^d$")
grid on; box on;
%% perform learned reference with pos/force (yes filter)
clear F_p Fe_p Fc_p;
ref.Fe = ref.Fe_;
controller = @(y,t,ref) controller_posforce(y,t,ref);
y_f = ode15s(@(t,y) ode(y,t,ref, controller),[0,T],y0);

for i = 1:length(y_f.x)
[F_p(i),Fe_p(i),Fc_p(i)] = force(y_f.y(:,i),y_f.x(i),ref,controller);
end

% plot: pos/force
figure(3); clf; 
subplot(4,1,1);hold on
plot(ref.t,ref.u,'k--')
plot(ref.t,ref.x)
plot(y_f.x,y_f.y(1,:))
ylabel("$x$ [m]")
grid on; box on;
legend("$u$","$x^d=x^i$","$x^p$")

subplot(4,1,2); hold on
plot(ref.t,ref.udot,'k--')
plot(ref.t,ref.xdot)
plot(y_f.x,y_f.y(2,:))
ylabel("$\dot{x}$ [m/s]")
grid on; box on;
legend("$\dot{u}$","$\dot{x}^d=\dot{x}^i$","$\dot{x}^p$")

subplot(4,1,3); hold on
plot(y.x,Fe)
plot(y_f.x,Fe_p)
% plot(ref.t,ref.Fe_,"k--")
ylabel("$F_e$ [N]")
grid on; box on;
legend("$F_e^d=F_e^i$","$F_e^f$","$\tilde{F}_e^d$")

subplot(4,1,4); hold on
plot(ref.t,ref.Fc)
plot(y_f.x,Fc_p)
ylabel("$F_c$ [N]")
grid on; box on;
legend("$F_c^d=F_c^i$","$F_c^f$","$\tilde{F}_e^d$")
%%
function ydot = ode(y,t,ref,controller)
m = 1;
xdot = y(2);

F = force(y,t,ref,controller);

xddot = F/m;
ydot = [xdot;xddot];

end

function [F,Fe,Fc] = force(y,t,ref,controller)
ke = 10^4;
be = 10;
x = y(1);
xdot = y(2);

if x>0
    Fe = 0;
else
    Fe = -ke*x-be*xdot;
end
    
Fc = controller(y,t,ref);

F = Fe + Fc;
end

function Fc = controller_impedance(y,t,ref)
kc = 40;
bc = 2*sqrt(kc);

x = y(1);
xdot = y(2);

xref = interp1(ref.t,ref.u,t);
xdotref = interp1(ref.t,ref.udot,t);

Fc = kc*(xref-x)+bc*(xdotref-xdot);

end

function Fc = controller_posforce(y,t,ref)
kc = 40;
bc = 2*sqrt(kc);

x = y(1);
xdot = y(2);

xref = interp1(ref.t,ref.x,t);
xdotref = interp1(ref.t,ref.xdot,t);
Fref = -interp1(ref.t,ref.Fe,t);

Fc = kc*(xref-x)+bc*(xdotref-xdot)+Fref;

end