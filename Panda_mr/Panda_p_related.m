function [p, pdot, Jaco, K] = Panda_p_related(q, qdot, l1, l2, l3, d1, d2, lf)

Jaco = zeros(3, 7);
K = zeros(3, 1);
% Jaco = sym(zeros(3, 7));
% K = sym(zeros(3, 1));

S1 = sin(q(1)); C1 = cos(q(1));
S2 = sin(q(2)); C2 = cos(q(2));
S3 = sin(q(3)); C3 = cos(q(3));
S4 = sin(q(4)); C4 = cos(q(4));
S5 = sin(q(5)); C5 = cos(q(5));
S6 = sin(q(6)); C6 = cos(q(6));

p_A1 = l1+l2;
p_B1 = S6*lf+C6*d2;
p_B2 = l3+S6*d2-C6*lf;
p_C1 = S5*p_B1;
p_C2 = C5*p_B1;
p_C3 = p_C2-d1;
p_D1 = p_A1+S4*p_C3+C4*p_B2;
p_D2 = d1-S4*p_B2+C4*p_C3;
p_D3 = p_D1-l1;
p_E1 = C3*p_D2-S3*p_C1;
p_E2 = S3*p_D2+C3*p_C1;
p_F1 = C2*p_E1+S2*p_D3;
px = C1*p_F1-S1*p_E2;
py = S1*p_F1+C1*p_E2;
pz = l1-S2*p_E1+C2*p_D3;

p = [px; py; pz];

J_A1 = pz-l1;
J_A2 = p_A1-p_D1;
J_A3 = p_D2-d1;
J_A4 = l3-p_B2;
J_B1 = C2*p_E2;
J_C1 = S3*J_A2;
J_C2 = C3*J_A2;
J_C3 = S2*J_A3+C2*J_C2;
J_D1 = S4*p_C1;
J_D2 = C4*p_C1;
J_D3 = S3*p_C2+C3*J_D2;
J_D4 = C3*p_C2-S3*J_D2;
J_D5 = S2*J_D1+C2*J_D3;
J_E1 = S5*J_A4;
J_E2 = C5*J_A4;
J_E3 = S4*J_E2+C4*p_B1;
J_E4 = S4*p_B1-C4*J_E2;
J_E5 = S3*J_E1+C3*J_E4;
J_E6 = S3*J_E4-C3*J_E1;
J_E7 = S2*J_E3-C2*J_E5;

Jaco(1, 1) = -py;
Jaco(2, 1) = px;
Jaco(1, 2) = C1*J_A1;
Jaco(2, 2) = S1*J_A1;
Jaco(3, 2) = -p_F1;
Jaco(1, 3) = -S1*p_E1-C1*J_B1;
Jaco(2, 3) = C1*p_E1-S1*J_B1;
Jaco(3, 3) = S2*p_E2;
Jaco(1, 4) = C1*J_C3-S1*J_C1;
Jaco(2, 4) = S1*J_C3+C1*J_C1;
Jaco(3, 4) = C2*J_A3-S2*J_C2;
Jaco(1, 5) = -S1*J_D4-C1*J_D5;
Jaco(2, 5) = C1*J_D4-S1*J_D5;
Jaco(3, 5) = S2*J_D3-C2*J_D1;
Jaco(1, 6) = S1*J_E6+C1*J_E7;
Jaco(2, 6) = S1*J_E7-C1*J_E6;
Jaco(3, 6) = S2*J_E5+C2*J_E3;

pdot_B1 = J_A4*qdot(6);
pdot_B2 = p_B1*qdot(6);
pdot_C1 = S5*pdot_B1+p_C2*qdot(5);
pdot_C2 = C5*pdot_B1-p_C1*qdot(5);
pdot_D1 = J_A3*qdot(4)+S4*pdot_C2+C4*pdot_B2;
pdot_D2 = J_A2*qdot(4)-S4*pdot_B2+C4*pdot_C2;
pdot_E1 = C3*pdot_D2-S3*pdot_C1-p_E2*qdot(3);
pdot_E2 = S3*pdot_D2+C3*pdot_C1+p_E1*qdot(3);
pdot_F1 = S2*pdot_D1+C2*pdot_E1+J_A1*qdot(2);
pxdot = C1*pdot_F1-S1*pdot_E2-py*qdot(1);
pydot = S1*pdot_F1+C1*pdot_E2+px*qdot(1);
pzdot = C2*pdot_D1-S2*pdot_E1-p_F1*qdot(2);

pdot = [pxdot; pydot; pzdot];

K_B1 = -pdot_B2*qdot(6);
K_B2 = pdot_B1*qdot(6);
K_C1 = (pdot_C2+C5*pdot_B1)*qdot(5)+S5*K_B1;
K_C2 = -(pdot_C1+S5*pdot_B1)*qdot(5)+C5*K_B1;
K_D1 = (pdot_D2-S4*pdot_B2+C4*pdot_C2)*qdot(4)+S4*K_C2+C4*K_B2;
K_D2 = -(pdot_D1+S4*pdot_C2+C4*pdot_B2)*qdot(4)-S4*K_B2+C4*K_C2;
K_E1 = -(pdot_E2+S3*pdot_D2+C3*pdot_C1)*qdot(3)-S3*K_C1+C3*K_D2;
K_E2 = (pdot_E1-S3*pdot_C1+C3*pdot_D2)*qdot(3)+S3*K_D2+C3*K_C1;
K_F1 = (pzdot-S2*pdot_E1+C2*pdot_D1)*qdot(2)+S2*K_D1+C2*K_E1;

K(1, 1) = -(pydot+S1*pdot_F1+C1*pdot_E2)*qdot(1)-S1*K_E2+C1*K_F1;
K(2, 1) = (pxdot-S1*pdot_E2+C1*pdot_F1)*qdot(1)+S1*K_F1+C1*K_E2;
K(3, 1) = -(pdot_F1+S2*pdot_D1+C2*pdot_E1)*qdot(2)-S2*K_E1+C2*K_D1;

end