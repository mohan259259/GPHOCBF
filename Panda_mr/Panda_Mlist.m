function Mlist = Panda_Mlist

T = diag([1, 1, 1, 1]);

p1 = [0; -0.0373; 0.2638];
theta1 = [0; 0; 0] * pi / 180;
Shape_1 = InertiaFrame(T, p1, theta1);
XYZ_1 = [+3.581e-03; +3.933e-02; +6.918e-02];
ABG_1 = [+1.57e+00; -5.59e-01; -1.05e-01] * pi / 180;

p2 = [0; 0.0372; 0.4020];
theta2 = [0; 0; 0] * pi / 180;
Shape_2 = InertiaFrame(T, p2, theta2);
XYZ_2 = [-3.425e-03; -3.370e-02; -4.031e-02];
ABG_2 = [+2.08e+01; +2.56e-01; -2.56e+01] * pi / 180;

p3 = [0.0414; 0.0282; 0.6148];
theta3 = [0; 0; 0] * pi / 180;
Shape_3 = InertiaFrame(T, p3, theta3);
XYZ_3 = [-1.413e-02; +1.107e-02; -3.227e-02];
ABG_3 = [-9.06e+00; +9.56e+00; -3.70e+01] * pi / 180;

p4 = [0.0411; -0.0275; 0.6825];
theta4 = [0; 0; 0] * pi / 180;
Shape_4 = InertiaFrame(T, p4, theta4);
XYZ_4 = [-1.209e-02; +4.002e-05; +7.089e-02];
ABG_4 = [+3.59e+00; -2.88e+01; -2.11e+01] * pi / 180;

p5 = [0.00017; 0.0372; 0.9279];
theta5 = [0; 0; 0] * pi / 180;
Shape_5 = InertiaFrame(T, p5, theta5);
XYZ_5 = [-1.239e-02; +3.841e-03; +6.666e-02];
ABG_5 = [+5.15e+00; +8.58e+00; -1.24e+01] * pi / 180;

p6 = [0.0422; -0.0053; 1.0466];
theta6 = [0; 0; 0] * pi / 180;
Shape_6 = InertiaFrame(T, p6, theta6);
XYZ_6 = [+1.769e-02; +1.585e-02; -2.771e-02];
ABG_6 = [-4.47e+00; +1.04e+01; +9.23e-02] * pi / 180;

p7 = [0.0928; 0.0023; 0.9211];
theta7 = [0; 0; 0] * pi / 180;
Shape_7 = InertiaFrame(T, p7, theta7);
XYZ_7 = [0; 0; 0];
ABG_7 = [0; 0; 0] * pi / 180;

p8 = [0.0877; 0; 0.8226];
theta8 = [-180; 0; -45] * pi / 180;
M8 = InertiaFrame(T, p8, theta8);

M1 = InertiaFrame(Shape_1, XYZ_1, ABG_1);
M2 = InertiaFrame(Shape_2, XYZ_2, ABG_2);
M3 = InertiaFrame(Shape_3, XYZ_3, ABG_3);
M4 = InertiaFrame(Shape_4, XYZ_4, ABG_4);
M5 = InertiaFrame(Shape_5, XYZ_5, ABG_5);
M6 = InertiaFrame(Shape_6, XYZ_6, ABG_6);
M7 = InertiaFrame(Shape_7, XYZ_7, ABG_7);

M01 = M1;
M12 = TransInv(M1) * M2;
M23 = TransInv(M2) * M3;
M34 = TransInv(M3) * M4;
M45 = TransInv(M4) * M5;
M56 = TransInv(M5) * M6;
M67 = TransInv(M6) * M7;
M78 = TransInv(M7) * M8;

Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67, M78);
end