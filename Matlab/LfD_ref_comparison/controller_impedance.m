function Fc = controller_impedance(y,t,ref,kc,bc)

x = y(1);
xdot = y(2);

uref = interp1(ref.t,ref.u,t);
udotref = interp1(ref.t,ref.udot,t);

Fc = kc*(uref-x)+bc*(udotref-xdot);

end