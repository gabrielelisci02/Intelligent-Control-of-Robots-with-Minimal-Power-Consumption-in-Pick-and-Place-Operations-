function [Q_final, EE_pose, errors, tau_tot] = IK_IS_Solver(robot, Target_Pos, tol, N)

% Inverse Kinematic solver
[Q_final, EE_pose, errors] = IK_solver(robot, Target_Pos, tol, N);

nsol = size(Q_final,1);
tau = zeros(nsol,6);
tau_stiff = zeros(nsol,6);
k = [80 50 50 30 25 20];

for i = 1:size(Q_final,1)
    q = Q_final(i,:);
    tau(i,:) = gravityTorque(robot, q);
    tau_stiff(i,:)= k.*q;
end

% Calculate the total torque for all solutions
total_gravity_Torque = sum(abs(tau), 2);
total_stiff_torque = sum(abs(tau_stiff),2);
tau_tot_matrix = abs(tau)+abs(tau_stiff);
tau_tot = sum(tau_tot_matrix, 2);

%% Output Table

jointNames = {'Joint1','Joint2','Joint3','Joint4','Joint5','Joint6'};

TorqueTable = array2table(tau_tot_matrix, ...
    'VariableNames', jointNames);

TorqueTable.Configuration = (1:nsol).';
TorqueTable.TotalTorque = tau_tot;

TorqueTable = movevars(TorqueTable, 'Configuration', 'Before', 'Joint1');

disp('--- Total torque for joint and configuration [Nm] ---')
disp(TorqueTable)

%% Optimal Configuration

[minTorque, bestIdx] = min(tau_tot);

fprintf('\n--- OPTIMAL CONFIGURATION ---\n');
fprintf('Optimal Configuration: %d\n', bestIdx);
fprintf('Optimal Torque = %.4f Nm\n', minTorque);

fprintf('optimal q  [q1 q2 q3 q4 q5 q6] =\n');
fprintf('[%.4f %.4f %.4f %.4f %.4f %.4f]\n', Q_final(bestIdx,:));

fprintf('EE pose [roll pitch yaw x y z] =\n');
fprintf('[%.4f %.4f %.4f %.4f %.4f %.4f]\n', EE_pose(bestIdx,:));

fprintf('Total Torque [Nm] =\n');
fprintf('[%.4f %.4f %.4f %.4f %.4f %.4f]\n', tau_tot_matrix(bestIdx,:));
