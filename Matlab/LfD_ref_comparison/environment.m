
function Fe = environment(y,d,ke,be)
x = y(1);
xdot = y(2);

if x>d
    Fe = 0;
else
    Fe = ke*(d-x)+be*-xdot;
end

end