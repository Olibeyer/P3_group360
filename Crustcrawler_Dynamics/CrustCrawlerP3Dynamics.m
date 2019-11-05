
% Known constants
syms g m1 m2 m3 mp L1 L2 Lc1 Lc2 Lc3 Lcp;

% Inertia tensor
I1 = sym('I1_',[3 3]); 
I2 = sym('I2_',[3 3]); 
I3 = sym('I3_',[3 3]);

% Theta as function of t
syms t1 t2 t3 dt1 dt2 dt3 ddt1 ddt2 ddt3;


% Rotational matrix
R2 = rotz(t1)*rotx(pi/2)*rotz(t2);
R3 = rotz(t1)*rotx(pi/2)*rotz(t2+t3);

% Z-axis of joint (Z1 is also global Z)
Z1 = [0; 0; 1];
Z2 = R2*Z1;
Z3 = R3*Z1;

% Angular velocity
W1 = Z1*dt1;    
W2 = W1+Z2*dt2; 
W3 = W2+Z3*dt3; 

% Vector from previous joint to center of mass
Sc1 = [0; 0; Lc1];
Sc2 = R2*[Lc2; 0; 0];
Sc3 = R3*[Lc3; 0; 0];
Scp = R3*[Lcp; 0; 0];

% Vector from previous joint to next joint
S1 = [0; 0; L1];
S2 = R2*[L2; 0; 0];

% Joint velocity
v1 = [0; 0; 0];
v2 = v1+cross(W1,S1);
v3 = v2+cross(W2,S2);

% Velocity in center of mass
vc2 = v2+cross(W2,Sc2);  
vc3 = v3+cross(W3,Sc3);  
vcp = v3+cross(W3,Scp);  

% Height at center of mass
h1 = Lc1;
h2 = L1+Lc2*sin(t2);
h3 = L1+L2*sin(t2)+Lc3*sin(t2+t3);
hp = L1+L2*sin(t2)+Lcp*sin(t2+t3);


% Kinetic and potential energy
% Ti = (1/2)*mi*dot(vci,vci) + (1/2)*dot(Wi,I*Wi)

T1 = (1/2)*dot(W1,I1*W1);
V1 = m1*g*h1;

T2 = (1/2)*m2*dot(vc2,vc2)+(1/2)*dot(W2,(R2*I2*R2')*W2);
V2 = m2*g*h2;

T3 = (1/2)*m3*dot(vc3,vc3)+(1/2)*dot(W3,(R3*I3*R3')*W3);
V3 = m3*g*h3;

% Payload
Tp = (1/2)*mp*dot(vcp,vcp);
Vp = mp*g*hp;

% Lagrangian equation
L = T1-V1+T2-V2+T3-V3+Tp-Vp;

%% tau1
y1 = diff(L,dt1);
tau1a = diff(y1,t1)*dt1+diff(y1,t2)*dt2+diff(y1,t3)*dt3+diff(y1,dt1)*ddt1+diff(y1,dt2)*ddt2+diff(y1,dt3)*ddt3;
tau1b = diff(L,t1);
tau1 = simplify(tau1a-tau1b);

% tau2
y2 = diff(L,dt2);
tau2a = diff(y2,t1)*dt1+diff(y2,t2)*dt2+diff(y2,t3)*dt3+diff(y2,dt1)*ddt1+diff(y2,dt2)*ddt2+diff(y2,dt3)*ddt3;
tau2b = diff(L,t2);
tau2 = simplify(tau2a-tau2b);

% tau3
y3 = diff(L,dt3);
tau3a = diff(y3,t1)*dt1+diff(y3,t2)*dt2+diff(y3,t3)*dt3+diff(y3,dt1)*ddt1+diff(y3,dt2)*ddt2+diff(y3,dt3)*ddt3;
tau3b = diff(L,t3);
tau3 = simplify(tau3a-tau3b);

%% Insert values

g = 9.81;

m1 = 0.22501148645;   m2 = 0.21285774246;   m3 = 0.28725035108;   mp = 0.55;  % mass of link, mp being mass of payload
L1 = 0.055;           L2 = 0.22;            Lcp = 0.15;                       % length of link
Lc1 = 0.05875;        Lc2 = 0.17433;        Lc3 = 0.12078;                    % length from joint to center of mass of link

% Inertia tensors
I1_1_1 = 0.08410695;    I1_1_2 = -0.00213140;   I1_1_3 = -0.00685242;
I1_2_1 = -0.00213140;   I1_2_2 = 0.11802166;    I1_2_3 = 0.00036003;
I1_3_1 = -0.00685242;   I1_3_2 = 0.00036003;    I1_3_3 = 0.11395469;

I2_1_1 = 0.63396931;    I2_1_2 = -0.01273805;   I2_1_3 = -0.01536619;
I2_2_1 = -0.01273805;   I2_2_2 = 0.25453628;    I2_2_3 = 0.26329519;
I2_3_1 = -0.01536619;   I2_3_2 = 0.26329519;    I2_3_3 = 0.44043802;

I3_1_1 = 0.25596532;    I3_1_2 = 0.00155757;    I3_1_3 = -0.01187757;
I3_2_1 = 0.00155757;    I3_2_2 = 0.32784813;    I3_2_3 = -0.02519485;
I3_3_1 = -0.01187757;   I3_3_2 = -0.02519485;   I3_3_3 = 0.10577771;


t1=0;    t2=0;     t3=0;     % angular position of joint
dt1=0;   dt2=1;    dt3=2;    % angular velocity of joint
ddt1=2;  ddt2=1;   ddt3=1;   % angular acceleration of joint


tau1final = vpa(subs(tau1));
tau2final = vpa(subs(tau2));
tau3final = vpa(subs(tau3));

taufinal = [tau1final;tau2final;tau3final]
