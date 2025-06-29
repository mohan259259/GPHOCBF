function gp_training()
% gp_training
% 示例: 离线训练 GP 来预测 d(x). 
% 这里仅演示写法, 实际需你准备 (X_train, Y_train)

% 1. 准备训练数据
% X_train = Nx6: [q1, q2, q1_dot, q2_dot, q1_ddot, q2_ddot]
% Y_train = Nx2: [d1, d2]  (如果要分开训练,则可以 Y_train(:,1), Y_train(:,2) 分别训练2个gprMdl)
% 本示例里没有真实数据. 你可在仿真或实验后保存.

N = 200; 
X_train = rand(N,6);   % 伪造一些随机数据
Y_train = [sin(X_train(:,1)),  0.2*X_train(:,2)];  % 伪造d1=sin(q1), d2=0.2*q2

% 2. 训练 GP (以 d1 为例)
gprMdl1 = fitrgp(X_train, Y_train(:,1), 'KernelFunction','squaredexponential',...
    'Standardize',true);
% 若要对 d2 也建模
gprMdl2 = fitrgp(X_train, Y_train(:,2), 'KernelFunction','squaredexponential',...
    'Standardize',true);

% 3. 保存模型
save('myGPModel.mat','gprMdl1','gprMdl2');
disp('GP training done, models saved to myGPModel.mat');
end
