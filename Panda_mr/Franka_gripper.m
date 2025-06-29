clear; clc;
addpath(genpath('..\'));

T = diag([1, 1, 1, 1]);

M1 = T;
m_1 = 4.3e-01;
Ix_1 = 3.583e-03 * m_1;
Iy_1 = 5.586e-04 * m_1;
Iz_1 = 3.285e-03 * m_1;
I1 = diag([Ix_1, Iy_1, Iz_1]);

p2 = [0.00025; -0.0520; 0.0569];
theta2 = [0.44; 0.16; -90.11] * pi / 180;
M2 = InertiaFrame(T, p2, theta2);
m_2 = 1e-01;
Ix_2 = 1e-03 * m_2;
Iy_2 = 1e-03 * m_2;
Iz_2 = 1e-03 * m_2;
I2 = diag([Ix_2, Iy_2, Iz_2]);

p3 = [0.00017; 0.0541; 0.0569];
theta3 = [-0.50; 0.10; -90.04] * pi / 180;
M3 = InertiaFrame(T, p3, theta3);
m_3 = 1e-01;
Ix_3 = 1e-03 * m_3;
Iy_3 = 1e-03 * m_3;
Iz_3 = 1e-03 * m_3;
I3 = diag([Ix_3, Iy_3, Iz_3]);

p4 = [0.00019; -0.0438; 0.0749];
theta4 = [0; -89.84; -90.27] * pi / 180;
M4 = InertiaFrame(T, p4, theta4);
m_4 = 5e-02;
Ix_4 = 1e-03 * m_4;
Iy_4 = 1e-03 * m_4;
Iz_4 = 1e-03 * m_4;
I4 = diag([Ix_4, Iy_4, Iz_4]);

p5 = [0.00027; 0.0466; 0.0750];
theta5 = [0; -89.95; -88.72] * pi / 180;
M5 = InertiaFrame(T, p5, theta5);
m_5 = 5e-02;
Ix_5 = 1e-03 * m_5;
Iy_5 = 1e-03 * m_5;
Iz_5 = 1e-03 * m_5;
I5 = diag([Ix_5, Iy_5, Iz_5]);

m_gri = m_1+m_2+m_3+m_4+m_5;
p = (m_2*p2+m_3*p3+m_4*p4+m_5*p5) / m_gri;
M0 = RpToTrans(eye(3), p);

M01 = TransInv(M0) * M1;
M02 = TransInv(M0) * M2;
M03 = TransInv(M0) * M3;
M04 = TransInv(M0) * M4;
M05 = TransInv(M0) * M5;

[R01, p01] = TransToRp(M01);
[R02, p02] = TransToRp(M02);
[R03, p03] = TransToRp(M03);
[R04, p04] = TransToRp(M04);
[R05, p05] = TransToRp(M05);

I1 = R01 * I1 * transpose(R01);
I2 = R02 * I2 * transpose(R02);
I3 = R03 * I3 * transpose(R03);
I4 = R04 * I4 * transpose(R04);
I5 = R05 * I5 * transpose(R05);

I1 = I1 + m_1 * (transpose(p01) * p01 * eye(3) - p01 * transpose(p01));
I2 = I2 + m_2 * (transpose(p02) * p02 * eye(3) - p02 * transpose(p02));
I3 = I3 + m_3 * (transpose(p03) * p03 * eye(3) - p03 * transpose(p03));
I4 = I4 + m_4 * (transpose(p04) * p04 * eye(3) - p04 * transpose(p04));
I5 = I5 + m_5 * (transpose(p05) * p05 * eye(3) - p05 * transpose(p05));

I_gri = I1 + I2 + I3 + I4 + I5;

p1 = [0.0877; -0.000012; 0.8963];
theta1 = [-180; 0; 135] * pi / 180;
Shape_1 = InertiaFrame(T, p1, theta1);
M_gri = Shape_1 * M0;
[R_gri, p_gri] = TransToRp(M_gri);

p7 = [0.0879; 0.00088; 0.9525];
theta7 = [0; 0; 0] * pi / 180;
Shape_7 = InertiaFrame(T, p7, theta7);
XYZ_7 = [+1.032e-02; +3.364e-03; +1.891e-02];
ABG_7 = [-4.21e+00; -9.00e+00; +5.85e+00] * pi / 180;
M7 = InertiaFrame(Shape_7, XYZ_7, ABG_7);
[R7, p7] = TransToRp(M7);
m_7 = 7.355e-01;
Ix_7 = 2.123e-02 * m_7;
Iy_7 = 1.755e-02 * m_7;
Iz_7 = 6.259e-03 * m_7;
I7 = diag([Ix_7, Iy_7, Iz_7]);

p_c = (m_gri*p_gri+m_7*p7) / (m_gri+m_7);
M_c = RpToTrans(eye(3), p_c);

Mc1 = TransInv(M_c) * M_gri;
Mc2 = TransInv(M_c) * M7;

[Rc1, pc1] = TransToRp(Mc1);
[Rc2, pc2] = TransToRp(Mc2);

I_gri = Rc1 * I_gri * transpose(Rc1);
I7 = Rc2 * I7 * transpose(Rc2);

I_gri = I_gri + m_gri * (transpose(pc1) * pc1 * eye(3) - pc1 * transpose(pc1));
I7 = I7 + m_7 * (transpose(pc2) * pc2 * eye(3) - pc2 * transpose(pc2));

I = I_gri + I7