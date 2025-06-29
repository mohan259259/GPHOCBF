function Slist = Panda_Slist

omg1 = [0; 0; 1];
omg2 = [0; 1; 0];
omg3 = [0; 0; 1];
omg4 = [0;-1; 0];
omg5 = [0; 0; 1];
omg6 = [0;-1; 0];
omg7 = [0; 0;-1];

q1 = [0; 0; 0];
q2 = [0; 0; 0.3330];
q3 = [0; 0; 0];
q4 = [0.0825; 0; 0.6490];
q5 = [0; 0; 0];
q6 = [0; 0; 1.0330];
q7 = [0.0880; 0; 1.0330];

v1 = -VecToso3(omg1) * q1;
v2 = -VecToso3(omg2) * q2;
v3 = -VecToso3(omg3) * q3;
v4 = -VecToso3(omg4) * q4;
v5 = -VecToso3(omg5) * q5;
v6 = -VecToso3(omg6) * q6;
v7 = -VecToso3(omg7) * q7;

S1 = [omg1; v1];
S2 = [omg2; v2];
S3 = [omg3; v3];
S4 = [omg4; v4];
S5 = [omg5; v5];
S6 = [omg6; v6];
S7 = [omg7; v7];

Slist = [S1, S2, S3, S4, S5, S6, S7];
end