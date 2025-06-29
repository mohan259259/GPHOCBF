%% Get Start State
action0.qstart = [0; 0; 0; -90; 0; 90; 45] * pi / 180;
action0.qend = [0; 0; 0; -120; 0; 120; 45] * pi / 180;    

%% Get Trajectory
action0.Tf = 5;
action0.n = action0.Tf / dt + 1;
action0.method = 5;
action0.qDesired = JointTrajectory(action0.qstart, action0.qend, ...
    action0.Tf, action0.n, action0.method);
qDesired = action0.qDesired;
N = action0.n;
qdotDesired = zeros(N, 7);
qddotDesired = zeros(N, 7);
for i = 1: N - 1
    qdotDesired(i + 1, :) = wrapToPi(qDesired(i + 1, :) - qDesired(i, :)) / dt;
    qddotDesired(i + 1, :) = (qdotDesired(i + 1, :) - qdotDesired(i, :)) / dt;
end
