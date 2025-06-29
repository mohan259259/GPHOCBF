function M = InertiaFrame(T, p, theta)
% T is shape frame
omghat_x = [1; 0; 0];
omghat_y = [0; 1; 0];
omghat_z = [0; 0; 1];
R = MatrixExp3(VecToso3(omghat_x * theta(1))) * MatrixExp3(VecToso3(omghat_y * theta(2))) ...
    * MatrixExp3(VecToso3(omghat_z * theta(3)));
M = T * RpToTrans(R, p);
end