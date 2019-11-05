close all;
I1 = [84106.95 -2131.40  -6852.42; 
      -2131.40 118021.66 360.03;
      -6852.42 360.03    113954.69];
  
I2 = [633969.31 -12738.05 -15366.19;
      -12738.05 254536.28 263295.19;
      -15366.19 263295.19 440438.02];

I3 = [255965.32 1557.57   -11877.57;
      1557.57   327848.13 -25194.85;
      -11877.57 -25194.85 105777.71];
  
conversion = 10^(-6);
I3 = I3.*conversion;
I2 = I2.*conversion;
I1 = I1.*conversion;


  
d1 = 0.055;
a3 = 0.22;
a4 = 0.15;

r = [0 0  -d1+0.05875];
r2 = [0.17433 0 0];
r3 = [0.12078 0 0];
%    Lc1 = 0.005875;  Lc2 = 0.17433;  Lc3 = 0.12078;              % length from joint to center of mass of link
%
% m1 = 0.1874;  m2 = 0.17226;  m3 = 0.18391;   mp = 0;    % mass of link, mp being mass of paylaod
m1 = 0.22501148645;  m2 = 0.21285774246;  m3 = 0.28725035108; mp = 0.55;
% alpha a d modified D&H
% Because we use Craig
Li(1) = Link('alpha', 0,'a', 0,      'd', d1,    'm', m1, 'I', I1,         'r', r, 'modified'); %rotational
Li(2) = Link('alpha', pi/2,'a', 0,   'd', 0,     'm', m2, 'I', I2,         'r', r2, 'modified'); %rotational
Li(3) = Link('alpha', 0,'a', a3,     'd', 0,     'm', m3, 'I', I3,         'r', r3, 'modified'); %rotational
Li(4) = Link('alpha', pi/2,'a', a4,  'd', 0,     'm', mp, 'I', zeros(3),  'r', zeros(3,1), 'modified'); %rotational
%L(5) = Link('alpha', 0,'a', 0,'d', 0, 'm', 0, 'modified', 'sym'); %rotational

threeDOF=SerialLink(Li, 'name', 'CrustCrawler');
threeDOF.plotopt = {'workspace' [-0.5,0.5,-0.5,0.5,-0.5,0.5]};


% syms t1 t2 t3 t4 dt1 dt2 dt3 ddt1 ddt2 ddt3
q =     [0   0   0  0];
qd =    [0   1   2  0];
qdd =   [2   1   1  0];

vpa(threeDOF.rne(q, qd, qdd)')

%threeDOF.teach();
%simplify(threeDOF.gravload(q))