%massmatrix:

H11=0.032 + 0.026*cos(2.0*t2 + t3) + 0.024*cos(2.0*t2)+ 3.3e-6*sin(2.0*t2)+ 0.026*cos(t3) - 1.0e-35*cos(2.0*t2 - 2.0*t3) + 8.4e-3*cos(2.0*t2 + 2.0*t3) + 5.6e-6*sin(2.0*t2 + 2.0*t3)
H12=-4.8e-6*cos(t2)- 5.8e-7*sin(t2) + 3.5e-7*cos(t2)*cos(t3)+ 2.7e-6*cos(t2)*sin(t3)+ 2.7e-6*cos(t3)*sin(t2) - 3.5e-7*sin(t2)*sin(t3)
H21=H12
H13=3.5e-7*cos(t2)*cos(t3) + 2.7e-6*cos(t2)*sin(t3) + 2.7e-6*cos(t3)*sin(t2) - 3.5e-7*sin(t2)*sin(t3)
H31=H13
H22= 0.064+0.052*cos(t3) 
H23=0.017+0.026*cos(t3)  
H32=H23
H33=0.017

mass=[H11,H12,H13;H21,H22,H23;H31,H32,H33]

