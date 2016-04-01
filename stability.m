%syms xddot yddot zddot phiddot thddot psiddot T m phi theta psi xdot ydot zdot g

syms x1 x2 x2dot x3 x4 x4dot x5 x6 x6dot x7 x8 x8dot x9 x10 x10dot x11 x12 x12dot T m g Ax Ay Az

%xddot = -g*[0;0;1] + (T/m)*[(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi));(sin(psi)*

xdots = -g*[0;0;1] + (T/m)*[(cos(x11)*sin(x9)*cos(x7)+sin(x11)*sin(x7));...
    (sin(x11)*sin(x9)*cos(x7)-cos(x11)*sin(x7));(cos(x9)*cos(x7))] - ...
    (1/m)*[Ax,0,0;0,Ay,0;0,0,Az]*[x2;x4;x6];

%x2dot = -g*0 + (T/m)*(cos(x11)*sin(x9)*cos(x7)+sin(x11)*sin(x7)) - (1/m)

x2dot = xdots(1);
x4dot = xdots(2);
x6dot = xdots(3);

df1 = [0,1;diff(x2dot,x1),diff(x2dot,x2)];
df2 = [0,1;diff(x4dot,x3),diff(x4dot,x4)];
df3 = [0,1;diff(x6dot,x5),diff(x6dot,x6)];

eigs1 = eig(df1)
eigs2 = eig(df2)
eigs3 = eig(df3)

syms Ixx Iyy Izz It omr Tphi Ttht Tpsi

% angdots = [((Iyy - Izz)*q*r/Ixx);((Izz - Ixx)*p*r/Iyy);((Ixx - Iyy)*p*q/Izz)] - ...
%     It*[(q/Ixx);(-p/Iyy);0]*omr + [(Tphi/Ixx);(Ttht/Iyy);(Tpsi/Izz)];
% 
% pdot = angdots(1);
% qdot = angdots(2);
% rdot = angdots(3);

