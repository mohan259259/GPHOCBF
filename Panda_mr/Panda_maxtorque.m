function tau_safe = Panda_maxtorque(tau)
tau_safe = tau;
max1 = 87;
max2 = 12;
if length(tau_safe) == 7
    for i = 1: 4
        if tau_safe(i) > max1
            tau_safe(i) = max1;
        end
        if tau_safe(i) < -max1
            tau_safe(i) = -max1;
        end
    end
    for i = 5: 7
        if tau_safe(i) > max2
            tau_safe(i) = max2;
        end
        if tau_safe(i) < -max2
            tau_safe(i) = -max2;
        end
    end
else
    error('Incorrect torque vector length.');
end
end