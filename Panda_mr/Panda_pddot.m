function pddot = Panda_pddot(q, qdot, qddot, l1, l2, l3, d1, d2, lf)

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

J_A1 = pz-l1;
J_A2 = p_A1-p_D1;
J_A3 = p_D2-d1;
J_A4 = l3-p_B2;

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

pddot_B1 = J_A4*qddot(6)-pdot_B2*qdot(6);
pddot_B2 = p_B1*qddot(6)+pdot_B1*qdot(6);
pddot_C1 = p_C2*qddot(5)+(pdot_C2+C5*pdot_B1)*qdot(5)+S5*pddot_B1;
pddot_C2 = -p_C1*qddot(5)-(pdot_C1+S5*pdot_B1)*qdot(5)+C5*pddot_B1;
pddot_D1 = J_A3*qddot(4)+(pdot_D2-S4*pdot_B2+C4*pdot_C2)*qdot(4)+S4*pddot_C2+C4*pddot_B2;
pddot_D2 = J_A2*qddot(4)-(pdot_D1+S4*pdot_C2+C4*pdot_B2)*qdot(4)-S4*pddot_B2+C4*pddot_C2;
pddot_E1 = -p_E2*qddot(3)-(pdot_E2+S3*pdot_D2+C3*pdot_C1)*qdot(3)-S3*pddot_C1+C3*pddot_D2;
pddot_E2 = p_E1*qddot(3)+(pdot_E1-S3*pdot_C1+C3*pdot_D2)*qdot(3)+S3*pddot_D2+C3*pddot_C1;
pddot_F1 = J_A1*qddot(2)+(pzdot-S2*pdot_E1+C2*pdot_D1)*qdot(2)+S2*pddot_D1+C2*pddot_E1;
pzddot = -p_F1*qddot(2)-(pdot_F1+S2*pdot_D1+C2*pdot_E1)*qdot(2)-S2*pddot_E1+C2*pddot_D1;
pxddot = -py*qddot(1)-(pydot+S1*pdot_F1+C1*pdot_E2)*qdot(1)-S1*pddot_E2+C1*pddot_F1;
pyddot = px*qddot(1)+(pxdot-S1*pdot_E2+C1*pdot_F1)*qdot(1)+S1*pddot_F1+C1*pddot_E2;

pddot = [pxddot; pyddot; pzddot];

end