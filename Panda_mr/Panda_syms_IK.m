function q = Panda_syms_IK(Tsb, l1, l2, l3, d1, d2, lf, q7, qa)

qa = qa(1: 6);
NAN = 90;
q4 = ones(1, 2) * NAN;
q6 = ones(1, 4) * NAN;
q5 = ones(1, 4) * NAN;
q1 = ones(1, 8) * NAN;
q2 = ones(1, 8) * NAN;
q3 = ones(1, 8) * NAN;
q4_test = zeros(1, 2);
q6_test = zeros(1, 4);
q5_test = zeros(1, 4);
q1_test = zeros(1, 8);
q2_test = zeros(1, 8);
q3_test = zeros(1, 8);
Q = ones(6, 8) * NAN;
Norm = ones(8, 1) * NAN;

qmax = [ 2.8973;  1.7628;  2.8973; -0.0698;  2.8973;  3.7525;  2.8973];
qmin = [-2.8973; -1.7628; -2.8973; -3.0718; -2.8973; -0.0175; -2.8973];

if qmin(7) > q7 || q7 > qmax(7)
    error('q7 out of joint limit!');
end

[R_EE, p_EE] = TransToRp(Tsb);
z_EE = R_EE(:, 3);
norm_O2O4 = sqrt(l2^2 + d1^2);
norm_O4O6 = sqrt(l3^2 + d1^2);
p7 = p_EE - lf*z_EE;
x6 = R_EE * [cos(q7-pi/4); -sin(q7-pi/4); 0];
p6 = p7 - d2*x6;
p2 = [0; 0; l1];
O2O6 = p6 - p2;
norm_O2O6 = norm(O2O6);

if norm_O2O6 < norm_O4O6-norm_O2O4 || norm_O2O6 > norm_O4O6+norm_O2O4
    error('Out of workspace!');
end

angle_O2O4O3 = atan2(l2, d1);
angle_HO4O6 = atan2(l3, d1);
angle_O2O4O6 = acos((norm_O2O4^2 + norm_O4O6^2 - norm_O2O6^2)/(2*norm_O2O4*norm_O4O6));
q4_test(2) = wrapToPi(angle_O2O4O3 + angle_HO4O6 - angle_O2O4O6);        % A1
q4_test(1) = wrapToPi(angle_O2O4O3 + angle_HO4O6 + angle_O2O4O6 - 2*pi); % A2

angle_O2O6O4 = acos((norm_O2O6^2 + norm_O4O6^2 - norm_O2O4^2)/(2*norm_O2O6*norm_O4O6));
angle_HO6O4 = pi/2 - angle_HO4O6;
angle_O2O6H = angle_O2O6O4 + angle_HO6O4;
y6 = -z_EE;
z6 = VecToso3(x6) * y6;
R6 = [x6, y6, z6];
O2O6_6 = transpose(R6) * O2O6;
x26_6 = O2O6_6(1);
y26_6 = O2O6_6(2);
Phi6 = atan2(y26_6, x26_6);
phi6 = asin(norm_O2O6 * cos(angle_O2O6H)/(sqrt(x26_6^2 + y26_6^2)));
angle_PO2O6 = 3*pi/2 - angle_O2O4O3 - angle_O2O4O6 - angle_O2O6O4;
angle_O2PO6 = angle_O2O6O4 + angle_O2O4O6 + angle_O2O4O3 - angle_O2O6H - pi/2;
norm_PO6 = norm_O2O6 * (sin(angle_PO2O6)/sin(angle_O2PO6));

for i_q4 = 1: 1
if qmin(4) <= q4_test(i_q4) && q4_test(i_q4) <= qmax(4)
    q4(i_q4) = q4_test(i_q4);
    q6_test(2*i_q4-1) = wrapTo2Pi(pi - phi6 - Phi6); % B1
    q6_test(2*i_q4) = wrapTo2Pi(phi6 - Phi6);        % B2
    
    for i_q6 = 2*i_q4-1: 2*i_q4
    if q6_test(i_q6) >= wrapTo2Pi(qmin(6))
        q6_test(i_q6) = q6_test(i_q6) - 2*pi; % include -1°
    end
    if q6_test(i_q6) <= qmax(6)
        q6(i_q6) = q6_test(i_q6);
        S6 = sin(q6(i_q6));
        C6 = cos(q6(i_q6));
        R6_5 = [C6, -S6,  0;
                 0,   0, -1;
                S6,  C6,  0];
        R5 = R6 * transpose(R6_5);
        z5 = R5(:, 3);
        O2P = O2O6 - norm_PO6*R6*[S6; C6; 0];
        y3 = Normalize(VecToso3(O2P)*O2O6);
        z3 = Normalize(O2P);
        x3 = VecToso3(y3) * z3;
        HO4 = p2 + l2*z3 + d1*x3 - p6 + l3*z5;
        O5S = transpose(R5) * HO4;
        q5_test(i_q6) = wrapToPi(-atan2(O5S(2), O5S(1)));
        if qmin(5) <= q5_test(i_q6) && q5_test(i_q6) <= qmax(5)
            q5(i_q6) = q5_test(i_q6);
            O2P(1) = ReduceError(O2P(1), 10);
            O2P(2) = ReduceError(O2P(2), 10);
            if O2P(1) == 0 && O2P(2) == 0
                q1_test(2*i_q6-1) = qa(1);
                q2_test(2*i_q6-1) = 0;
                q1_test(2*i_q6) = qa(1);
                q2_test(2*i_q6) = 0;
            else
                q1_test(2*i_q6-1) = wrapToPi(atan2(O2P(2), O2P(1)));
                q2_test(2*i_q6-1) = wrapToPi(acos(O2P(3)/norm(O2P)));  % C1
                q1_test(2*i_q6) = wrapToPi(atan2(-O2P(2), -O2P(1)));
                q2_test(2*i_q6) = wrapToPi(-acos(O2P(3)/norm(O2P))); % C2
            end
            for i_q12 = 2*i_q6-1: 2*i_q6
            if qmin(1) <= q1_test(i_q12) && q1_test(i_q12) <= qmax(1) && qmin(2) <= q2_test(i_q12) && q2_test(i_q12) <= qmax(2)
                q1(i_q12) = q1_test(i_q12);
                q2(i_q12) = q2_test(i_q12);
                S1 = sin(q1(i_q12));
                C1 = cos(q1(i_q12));
                S2 = sin(q2(i_q12));
                C2 = cos(q2(i_q12));
                R2 = [C1*C2, -C1*S2, -S1;
                      C2*S1, -S1*S2,  C1;
                        -S2,    -C2,   0];
                x3_2 = transpose(R2) * x3;
                q3_test(i_q12) = wrapToPi(atan2(x3_2(3), x3_2(1)));
                if qmin(3) <= q3_test(i_q12) && q3_test(i_q12) <= qmax(3)
                    q3(i_q12) = q3_test(i_q12);
                    Q(:, i_q12) = [q1(i_q12); q2(i_q12); q3(i_q12); q4(i_q4); q5(i_q6); q6(i_q6)];
                    Norm(i_q12) = norm(wrapToPi(qa - Q(:, i_q12)));
                end
            end
            end
        end
    end
    end
end
end

[delta, pos] = min(Norm);
if delta > 10
    error('Out of joint limit!');
else
    q = [Q(:, pos); q7];
end
end