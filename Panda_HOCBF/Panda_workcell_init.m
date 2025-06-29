function handles = Panda_workcell_init(vrep, id)
robot_name = 'Franka';
handles = struct('id', id);
%% arm joints
armJoints = -ones(1, 7);
obstacle = -1;
robot = -1;
for i = 1: 7 % get handles of joints
    [res, armJoints(i)] = vrep.simxGetObjectHandle(id, ...
        [robot_name, '_joint', num2str(i)], vrep.simx_opmode_blocking);
    vrchk(vrep, res);
end
[~, obstacle] = vrep.simxGetObjectHandle(id, 'Sphere', vrep.simx_opmode_blocking);
[~, robot] = vrep.simxGetObjectHandle(id, 'Franka', vrep.simx_opmode_blocking);
[~, tool] = vrep.simxGetObjectHandle(id, 'Tool', vrep.simx_opmode_blocking);
handles.armJoints = armJoints;
handles.obstacle = obstacle;
handles.safe_region = obstacle;
handles.robot = robot;
handles.tool = tool;

%% streaming joint positions and velocities
for i = 1: 7
    vrep.simxGetJointPosition(id, armJoints(i), vrep.simx_opmode_streaming); % joint position
    vrchk(vrep, res, true);
    vrep.simxGetObjectFloatParameter(id, armJoints(i), 2012, vrep.simx_opmode_streaming); % joint velocity
    vrchk(vrep, res, true);
end
% pos = vrep.simxGetObjectPosition(id, obstacle, robot, vrep.simx_opmode_streaming);
% vrchk(vrep, res, true);
% Make sure that all streaming data has reached the client at least once
vrep.simxGetPingTime(id);
end