clear; clc;
addpath(genpath('..\'));

T = diag([1, 1, 1, 1]);

M1 = T;
m_1 = 2.3e-01;
Ix_1 = 3.425e-03 * m_1;
Iy_1 = 1.370e-03 * m_1;
Iz_1 = 2.329e-03 * m_1;
I1 = diag([Ix_1, Iy_1, Iz_1]);

p2 = [0.00025; -0.0520; 0.0569];
theta2 = [0.44; 0.16; -90.11] * pi / 180;
M2 = InertiaFrame(T, p2, theta2);
m_2 = 1.25e-01;
Ix_2 = 1e-03 * m_2;
Iy_2 = 1e-03 * m_2;
Iz_2 = 1e-03 * m_2;
I2 = diag([Ix_2, Iy_2, Iz_2]);

p3 = [0.00017; 0.0541; 0.0569];
theta3 = [-0.50; 0.10; -90.04] * pi / 180;
M3 = InertiaFrame(T, p3, theta3);
m_3 = 1.25e-01;
Ix_3 = 1e-03 * m_3;
Iy_3 = 1e-03 * m_3;
Iz_3 = 1e-03 * m_3;
I3 = diag([Ix_3, Iy_3, Iz_3]);

p4 = [0.00019; -0.0438; 0.0749];
theta4 = [0; -89.84; -90.27] * pi / 180;
M4 = InertiaFrame(T, p4, theta4);
m_4 = 1.25e-01;
Ix_4 = 1e-03 * m_4;
Iy_4 = 1e-03 * m_4;
Iz_4 = 1e-03 * m_4;
I4 = diag([Ix_4, Iy_4, Iz_4]);

p5 = [0.00027; 0.0466; 0.0750];
theta5 = [0; -89.95; -88.72] * pi / 180;
M5 = InertiaFrame(T, p5, theta5);
m_5 = 1.25e-01;
Ix_5 = 1e-03 * m_5;
Iy_5 = 1e-03 * m_5;
Iz_5 = 1e-03 * m_5;
I5 = diag([Ix_5, Iy_5, Iz_5]);

p = (m_2*p2+m_3*p3+m_4*p4+m_5*p5) / (m_1+m_2+m_3+m_4+m_5);
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

I = I1 + I2 + I3 + I4 + I5;