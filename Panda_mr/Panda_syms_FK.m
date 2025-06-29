function Tsb = Panda_syms_FK(q, l1, l2, l3, d1, d2, lf)

S1 = sin(q(1)); C1 = cos(q(1));
S2 = sin(q(2)); C2 = cos(q(2));
S3 = sin(q(3)); C3 = cos(q(3));
S4 = sin(q(4)); C4 = cos(q(4));
S5 = sin(q(5)); C5 = cos(q(5));
S6 = sin(q(6)); C6 = cos(q(6));
S7 = sin(q(7)-pi/4); C7 = cos(q(7)-pi/4);

o_A1 = S6*S7; o_A2 = S6*C7;
o_A3 = C6*S7; o_A4 = C6*C7;
o_B1 = S5*S7+C5*o_A4; o_B2 = C5*S7-S5*o_A4;
o_B3 = S5*C7-C5*o_A3; o_B4 = C5*C7+S5*o_A3;
o_C1 = S4*o_B1+C4*o_A2; o_C2 = C4*o_B1-S4*o_A2;
o_C3 = S4*o_B3-C4*o_A1; o_C4 = C4*o_B3+S4*o_A1;
o_D1 = C3*o_C2+S3*o_B2; o_D2 = S3*o_C2-C3*o_B2;
o_D3 = C3*o_C4+S3*o_B4; o_D4 = S3*o_C4-C3*o_B4;
o_E1 = S2*o_C1+C2*o_D1; o_E2 = S2*o_C3+C2*o_D3;

nx = C1*o_E1-S1*o_D2;
ny = S1*o_E1+C1*o_D2;
nz = C2*o_C1-S2*o_D1;
ox = C1*o_E2-S1*o_D4;
oy = S1*o_E2+C1*o_D4;
oz = C2*o_C3-S2*o_D3;

o_a1 = S5*S6; o_a2 = C5*S6;
o_b1 = S4*C6+C4*o_a2; o_b2 = C4*C6-S4*o_a2;
o_c1 = C3*o_b1-S3*o_a1; o_c2 = S3*o_b1+C3*o_a1;
o_e1 = C2*o_c1-S2*o_b2;

ax = C1*o_e1-S1*o_c2;
ay = S1*o_e1+C1*o_c2;
az =-S2*o_c1-C2*o_b2;

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

Tsb = [nx,   ox,   ax,   px;
       ny,   oy,   ay,   py;
       nz,   oz,   az,   pz;
        0,    0,    0,    1];
end