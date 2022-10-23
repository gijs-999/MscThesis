clear all; Lines = lines(20);
%% set parameters
ke = 10^4;
be = 100;
kc = 40;
bc = 2*sqrt(kc);
d = 0; % position of contact surface

%% generate demonstration reference
k = 4;
x0 = k*0.35;
xdot0 = 0;

T = 2;
n = T*1000;
ref.t = (0:n-1)/(n-1)*T ;

ref.uddot = [k*-1.25*ones(0.2*n,1);
            zeros(0.2*n,1);
             k*1.25*ones(0.2*n,1);
             zeros(0.4*n,1)];
ref.udot = xdot0+cumtrapz(ref.t,ref.uddot);
ref.u = x0+cumtrapz(ref.t,ref.udot);

% figure(102); clf; hold on;

% plot(ref.t,ref.u);
% plot(ref.t,ref.udot);
% plot(ref.t,ref.uddot);

%% perform demonstration, save data, and plot
y0 = [ref.u(1);ref.udot(1)];

ref_loc = ref;
ref_loc.udot = ref_loc.udot*0;
Fc  = @(t,y) controller_impedance(y,t,ref_loc,kc,bc);
Fe  = @(t,y) environment(y,d,ke,be);
ode = @(t,y) EoM(y,Fc(t,y),Fe(t,y));

sol = ode15s(ode,[0,T],y0);

for i = 1:length(sol.x) % find forces coerelating to states in each timestep
    sol.Fe(i) = Fe(sol.x(i),sol.y(:,i));
    sol.Fc(i) = Fc(sol.x(i),sol.y(:,i));
end

% first order filter on Fe
kf = 10;
options = odeset("MaxStep",1e-2);
sol_filt = ode15s(@(t,Fe_) kf*(interp1(sol.x, sol.Fe,t) - Fe_),[0,T],sol.Fe(1),options) ;

% interpolate so it matches the time during the reference
ref.Fe_  = interp1(sol_filt.x,sol_filt.y,ref.t);
ref.Fe   = interp1(sol.x,sol.Fe,ref.t);
ref.Fc   = interp1(sol.x,sol.Fc,ref.t);
ref.x    = interp1(sol.x,sol.y(1,:),ref.t);
ref.xdot = interp1(sol.x,sol.y(2,:),ref.t);

clear sol sol_filt
%% apply reference spreading
T_inter = 0.5;
T_delay = 0;
% find impact
i = min(find(ref.x<=d));
i_p = min(find(ref.t>=ref.t(i)+T_inter));

% extend ante and post reference
ref1 = ref;
ref2 = ref;

% ref1.udot(i:end) = ref1.udot(i-1);
% ref2.udot(1:i_p) = ref2.udot(i_p+1);
% ref1.u(i:end) = ref1.u(i-1)+cumtrapz(ref1.t(i:end),ref1.udot(i:end));
% ref2.u(1:i_p) = ref2.u(i_p+1)-cumtrapz(ref2.t(1:i_p),ref2.udot(1:i_p));
% 
% ref1.xdot(i:end) = ref1.xdot(i-1);
% ref2.xdot(1:i_p) = ref2.xdot(i_p+1);
% ref1.x(i:end) = ref1.x(i-1)+cumtrapz(ref1.t(i:end),ref1.xdot(i:end));
% ref2.x(1:i_p) = ref2.x(i_p+1)-cumtrapz(ref2.t(1:i_p),ref2.xdot(1:i_p));
% 
% ref1.Fe_(i:end) = ref1.Fe_(i-1);
% ref2.Fe_(1:i_p) = ref2.Fe_(i_p+1);
% ref1.Fe(i:end) = ref1.Fe(i-1);
% ref2.Fe(1:i_p) = ref2.Fe(i_p+1);
% ref2.t = ref.t;

ref1.udot=0*ref1.udot; ref2.udot=0*ref2.udot;
ref1.xdot=0*ref1.xdot; ref2.xdot=0*ref2.xdot;

%% plot: ante/post reference
f{2} = figure(2); clf; 

subplot(4,1,2);hold on
plot(ref.t,ref.u,'k--')
plot(ref1.t,ref1.u,'--',"color",Lines(1,:))
plot(ref2.t,ref2.u,'--',"color",Lines(2,:))
plot(ref.t,ref.x,'k')
plot(ref1.t,ref1.x,'b',"color",Lines(1,:))
plot(ref2.t,ref2.x,'r',"color",Lines(2,:))
ylabel("position [m]")
grid on; box on;
legend("$y$","$y^-$","$y^+$","$x$","$x^-$","$x^+$",'NumColumns',2)

subplot(4,1,3); hold 
plot(ref.t,ref.udot,'k--')
plot(ref1.t,ref1.udot,'b--',"color",Lines(1,:))
plot(ref2.t,ref2.udot,'r--',"color",Lines(2,:))
plot(ref.t,ref.xdot,'k')
plot(ref1.t,ref1.xdot,'b',"color",Lines(1,:))
plot(ref2.t,ref2.xdot,'r',"color",Lines(2,:))
ylabel("velocity [m/s]")
grid on; box on;
legend("$\dot{y}$","$\dot{y}^-$","$\dot{u}^+$","$\dot{x}$","$\dot{x}^-$","$\dot{x}^+$",'NumColumns',2)

subplot(4,1,4); hold on
plot(ref.t,ref.Fe_,'k')
plot(ref1.t,ref1.Fe_,'b',"color",Lines(1,:))
plot(ref2.t,ref2.Fe_,'r',"color",Lines(2,:))

ylabel({"[N]","contact force"})
grid on; box on;
legend("$\tilde{F}_e^d$","$\tilde{F}_e^{d-}$","$\tilde{F}_e^{d+}$")

% subplot(4,1,4); hold on
% plot(ref.t,ref.Fc)
% ylabel("control force [N]")
% legend("$F_c^d$")
% grid on; box on;
xlabel("time [s]")
%% change contact surface position
d = 0.1;
%% apply learned reference: impedance
y0 = [ref.u(1);ref.udot(1)];

% ante impact
Fc  = @(t,y) controller_impedance(y,t,ref1,kc,bc);
Fe  = @(t,y) environment(y,d,ke,be);
ode = @(t,y) EoM(y,Fc(t,y),Fe(t,y));
sol1 = ode15s(ode,[0,T],y0);

for i = 1:length(sol1.x) % find forces coerelating to states in each timestep
    sol1.Fe(i) = Fe(sol1.x(i),sol1.y(:,i));
    sol1.Fc(i) = Fc(sol1.x(i),sol1.y(:,i));
end

% post impact
i1 = min(find(sol1.y(1,:)<=d));
i1 = min(find(sol1.x>=sol1.x(i1)+T_delay));
i_p = min(find(sol1.x>=sol1.x(i1)+T_inter));
Fc  = @(t,y) controller_impedance(y,t,ref2,kc,bc);
Fe  = @(t,y) environment(y,d,ke,be);
ode = @(t,y) EoM(y,Fc(t,y),Fe(t,y));
sol2 = ode15s(ode,[sol1.x(i1),T],sol1.y(:,i1));
i2 = min(find(sol2.x>sol1.x(i1)));

for i = 1:length(sol2.x) % find forces coerelating to states in each timestep
    sol2.Fe(i) = Fe(sol2.x(i),sol2.y(:,i));
    sol2.Fc(i) = Fc(sol2.x(i),sol2.y(:,i));
end

% combine results
sol.x = [sol1.x(1:i1-1),sol2.x(i2:end)];
sol.y = [sol1.y(:,1:i1-1),sol2.y(:,i2:end)];
sol.Fe = [sol1.Fe(:,1:i1-1),sol2.Fe(:,i2:end)];
sol.Fc = [sol1.Fc(:,1:i1-1),sol2.Fc(:,i2:end)];

% interpolate so it matches the time during the reference
ref.Fe_i   = interp1(sol.x,sol.Fe,ref.t);
ref.Fc_i   = interp1(sol.x,sol.Fc,ref.t);
ref.x_i    = interp1(sol.x,sol.y(1,:),ref.t);
ref.xdot_i = interp1(sol.x,sol.y(2,:),ref.t);

clear sol sol1 sol2

%% apply learned reference: pos/force (without filter)
y0 = [ref.u(1);ref.udot(1)];

% ante impact
Fc  = @(t,y) controller_pos(y,t,ref1,kc,bc);
Fe  = @(t,y) environment(y,d,ke,be);
ode = @(t,y) EoM(y,Fc(t,y),Fe(t,y));
sol1 = ode15s(ode,[0,T],y0);

for i = 1:length(sol1.x) % find forces coerelating to states in each timestep
    sol1.Fe(i) = Fe(sol1.x(i),sol1.y(:,i));
    sol1.Fc(i) = Fc(sol1.x(i),sol1.y(:,i));
end

% post impact
i1 = min(find(sol1.y(1,:)<=d));
i1 = min(find(sol1.x>=sol1.x(i1)+T_delay));
i_p = min(find(sol1.x>=sol1.x(i1)+T_inter));
Fc  = @(t,y) controller_pos(y,t,ref2,kc,bc);
Fe  = @(t,y) environment(y,d,ke,be);
ode = @(t,y) EoM(y,Fc(t,y),Fe(t,y));
sol2 = ode15s(ode,[sol1.x(i1),T],sol1.y(:,i1),options);
i2 = min(find(sol2.x>sol1.x(i1)));

for i = 1:length(sol2.x) % find forces coerelating to states in each timestep
    sol2.Fe(i) = Fe(sol2.x(i),sol2.y(:,i));
    sol2.Fc(i) = Fc(sol2.x(i),sol2.y(:,i));
end

% combine results
sol.x = [sol1.x(1:i1-1),sol2.x(i2:end)];
sol.y = [sol1.y(:,1:i1-1),sol2.y(:,i2:end)];
sol.Fe = [sol1.Fe(:,1:i1-1),sol2.Fe(:,i2:end)];
sol.Fc = [sol1.Fc(:,1:i1-1),sol2.Fc(:,i2:end)];

% interpolate so it matches the time during the reference
ref.Fe_f   = interp1(sol.x,sol.Fe,ref.t);
ref.Fc_f   = interp1(sol.x,sol.Fc,ref.t);
ref.x_f    = interp1(sol.x,sol.y(1,:),ref.t);
ref.xdot_f = interp1(sol.x,sol.y(2,:),ref.t);

clear sol sol1 sol2


%% plot: demonstration
f{1} = figure(1); clf; 
subplot(4,1,1);hold on
plot(ref.t,ref.u,'k--')
plot(ref.t,ref.x,"color",lines(1))
ylabel("position [m]")
grid on; box on;
legend("$y$","$x^d$")

subplot(4,1,2); hold 
plot(ref.t,ref.udot,'k--')
plot(ref.t,ref.xdot,"color",lines(1))
ylabel("velocity [m/s]")
grid on; box on;
legend("$\dot{y}$","$\dot{x}^d$")

subplot(4,1,3); hold on
plot(ref.t,ref.Fe)
% plot(ref.t,ref.Fe_)
ylabel({"[N]","contact force"})
grid on; box on;

legend("$F_e^d$","$\tilde{F}_e^d$")
subplot(4,1,4); hold on
plot(ref.t,ref.Fc)

ylabel({"[N]","control force"})
legend("$F_c^d$")
grid on; box on;
xlabel("time [s]")

 %% plot: filter vs unfiltered
% figure(2); clf
% 
% subplot(4,1,3); hold on
% plot(ref.t,ref.Fe)
% plot(ref.t,ref.Fe_)
% ylabel("contact force [N]")
% grid on; box on;
% legend("$F_e^d$","$\tilde{F}_e^d$")
% xlabel("time [s]")

% %% plot: lfd controller without RS (no filter)
% figure(3); clf; 
% subplot(4,1,1);hold on
% plot(ref.t,ref.u,'k--')
% plot(ref.t,ref.x,'k:')
% plot(ref.t,ref.x_i)
% plot(ref.t,ref.x_f)
% % plot(ref.t,ref.x_f_)
% plot(ref.t,ref.x,'k:')
% ylabel("position [m]")
% grid on; box on;
% legend("$u$","$x^d$","$x^i$","$x^f$")%,"$x^{\tilde{f}}$")
% 
% subplot(4,1,2); hold 
% plot(ref.t,ref.udot,'k--')
% plot(ref.t,ref.xdot,'k:')
% plot(ref.t,ref.xdot_i)
% plot(ref.t,ref.xdot_f)
% % plot(ref.t,ref.xdot_f_)
% plot(ref.t,ref.xdot,'k:')
% ylabel("velocity [m/s]")
% grid on; box on;
% legend("$\dot{u}$","$\dot{x}^d$","$\dot{x}^i$","$\dot{x}^f$")%,"$\dot{x}^{\tilde{f}}$")
% 
% subplot(4,1,3); hold on
% plot(ref.t,ref.Fe,'k:')
% plot(ref.t,ref.Fe_i)
% plot(ref.t,ref.Fe_f)
% plot(ref.t,ref.Fe,'k:')
% ylabel("contact force [N]")
% grid on; box on;
% legend("$F_e^d$","$F_e^i$","$F_e^f$")
% 
% subplot(4,1,4); hold on
% plot(ref.t,ref.Fc,'k:')
% plot(ref.t,ref.Fc_i)
% plot(ref.t,ref.Fc_f)
% plot(ref.t,ref.Fc,'k:')
% ylabel("control force [N]")
% grid on; box on;
% legend("$F_c^d$","$F_c^i$","$F_c^f$")
% xlabel("time [s]")

%% plot: lfd controller
f{3} = figure(3); clf; 
subplot(4,1,1);hold on
plot(ref.t,ref.u,'k--')
plot(ref.t,ref.x,'k:')
plot(ref.t,ref.x_i,"color",lines(1))
plot(ref.t,ref.x_f,"color",Lines(2,:))
% plot(ref.t,ref.x_f_)
plot(ref.t,ref.x,'k:')
ylabel("position [m]")
grid on; box on;
legend("$y$","$x^d$","$x^i$","$x^{\tilde{f}}$")

subplot(4,1,2); hold 
plot(ref.t,ref.udot,'k--')
plot(ref.t,ref.xdot,'k:')
plot(ref.t,ref.xdot_i,"color",lines(1))
plot(ref.t,ref.xdot_f,"color",Lines(2,:))
% plot(ref.t,ref.xdot_f)
plot(ref.t,ref.xdot,'k:')
ylabel("velocity [m/s]")
grid on; box on;
legend("$\dot{y}$","$\dot{x}^d$","$\dot{x}^i$","$\dot{x}^{\tilde{f}}$")

subplot(4,1,3); hold on
plot(ref.t,ref.Fe,'k:')
plot(ref.t,ref.Fe_i,"color",lines(1))
plot(ref.t,ref.Fe_f,"color",Lines(2,:))
plot(ref.t,ref.Fe,'k:')
ylabel({"[N]","contact force"})
grid on; box on;
legend("$F_e^d$","$F_e^i$","$F_e^{\tilde{f}}$")

subplot(4,1,4); hold on
plot(ref.t,ref.Fc,'k:')
plot(ref.t,ref.Fc_i,"color",lines(1))
plot(ref.t,ref.Fc_f,"color",Lines(2,:))
plot(ref.t,ref.Fc,'k:')
ylabel({"[N]","control force"})
grid on; box on;
legend("$F_c^d$","$F_c^i$","$F_c^{\tilde{f}}$")
xlabel("time [s]")

%%
w = 700;
figure(1)
f{1}.Position =[10 10 w 440] ;
figure(2)
f{2}.Position =[10 10 w 440] ;
figure(3)
f{3}.Position =[10 10 w 440] ;

folder = "../../Graphics/"
% print(f{1},folder+'foo.png','-dpng','-r600');

% exportgraphics(f{1},folder+'1d_demonstration.pdf','Resolution',600)
% exportgraphics(f{2},folder+'1d_RS.pdf','Resolution',600)
exportgraphics(f{3},folder+'1d_learned_w='+string(d)+'_novel_nors.pdf','Resolution',600)