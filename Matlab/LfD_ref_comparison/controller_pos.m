function Fc = controller_pos(y,t,ref,kc,bc)

x = y(1);
xdot = y(2);

xref = interp1(ref.t,ref.x,t);
xdotref = interp1(ref.t,ref.xdot,t);
Fref = -interp1(ref.t,ref.Fe_,t);

Fc = kc*(xref-x)+bc*(xdotref-xdot)+Fref;

end