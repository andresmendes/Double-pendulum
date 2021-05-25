%% Double pendulum - Equations
% Derivation of the equations of motion of a double pendulum.
%
%%
clear ; close all ; clc

%% Symbolic variables
syms rA rB mA mB g thA thB dthA dthB ddthA ddthB

%% Equations of motion
EQ1 = mB*(-rA^2*ddthA - rA*rB*ddthB*cos(thA-thB) - rA*rB*dthB^2*sin(thA-thB)) - (mA+mB)*g*rA*sin(thA) - mA*rA^2*ddthA;
EQ2 = -mB*g*rB*sin(thB) - mB*(rA*rB*ddthA*cos(thA-thB) - rA*rB*dthA^2*sin(thA-thB)) - mB*rB^2*ddthB;

%% Solving fot ddthA and ddthB
sol = solve(EQ1==0,EQ2==0,[ddthA ddthB]);

ddth1 = simplify(sol.ddthA);
ddth2 = simplify(sol.ddthB);

disp(ddth1)
disp(ddth2)