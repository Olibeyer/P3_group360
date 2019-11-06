close all;

d1 = 0.055;
a3 = 0.22;
a4 = 0.15;

% alpha a d modified D&H
% Because we use Craig
L(1) = Link('alpha', 0,'a', 0,'d', d1,'modified'); %rotational
L(2) = Link('alpha', pi/2,'a', 0,'d', 0, 'offset',pi/2, 'modified'); %rotational
L(3) = Link('alpha', 0 ,'a', a3,'d', 0,'modified'); %rotational
L(4) = Link('alpha', pi/2,'a', a4,'d', 0,'modified'); %rotational

threeDOF=SerialLink(L, 'name', 'CrustCrawler');
threeDOF.plotopt = {'workspace' [-0.5,0.5,-0.5,0.5,-0.5,0.5]};
%Lets test the movement
threeDOF.teach('eul');