function [Q_final, EE_pose, errors] = IK_solver(robot, Target_Pos, tol, N)

lim2 = robot.getBody("body2").Joint.PositionLimits;
lim3 = robot.getBody("body3").Joint.PositionLimits;
lim5 = robot.getBody("body5").Joint.PositionLimits;

q2_vals = linspace(lim2(1), lim2(2), N);
q2_vals = [0, q2_vals];
q3_vals = linspace(lim3(1), lim3(2), N);
q3_vals = [0, q3_vals]; 
q5_vals = linspace(lim5(1), lim5(2), N);
q5_vals = [0, q5_vals];

% For picking
q = [0 0 0 0 0 0];

% For placing
% q = [0.0000 0.1812 0.4499 0.0000 0.5859 0.0000] % it's the pick_target_Pos

errors  = [];
EE_pose = [];
Q_final = [];

for q2 = q2_vals
    for q3 = q3_vals
        for q5 = q5_vals
            q(1) = 0;
            q(2) = q2;
            q(3) = q3;
            q(4) = 0;
            q(5) = q5;
            q(6) = 0;

            Tsol = getTransform(robot, q, "tool");
            Rsol = Tsol(1:3,1:3); 
            rpy = rotm2eul(Rsol, 'XYZ'); 
            psol = Tsol(1:3,4)';
            ee_pose = [rpy psol];
            zTool = Tsol(1:3, 3);

            if zTool(3) < -1 || zTool(3) > -0.9
                continue
            end 
            err = norm(psol - Target_Pos);
            if err < tol
                errors  = [errors; err];
                EE_pose = [EE_pose; ee_pose];
                Q_final = [Q_final; q];
            end
        end
    end
end

fprintf("Numero di soluzioni trovate: %d\n", size(Q_final,1));

for s = 1:size(Q_final,1)
    fprintf("\nSoluzione %d\n", s);
    fprintf("q finale [q1 q2 q3 q4 q5 q6] = [%.4f %.4f %.4f %.4f %.4f %.4f]\n", Q_final(s,:));
    fprintf("EE pose [roll pitch yaw x y z] = [%.4f %.4f %.4f %.4f %.4f %.4f]\n", EE_pose(s,:));
    fprintf("Errore posizione = %.6f m\n", errors(s));
    
    figure;
    robot.show(Q_final(s,:), 'Frames', 'on', 'PreservePlot', false);
    hold on;
    plot3(Target_Pos(1), Target_Pos(2), Target_Pos(3),'ro', 'MarkerSize', 10, 'LineWidth', 2);
    hold off, axis equal, grid on;
    xlabel('X'), ylabel('Y'), zlabel('Z');
    view(135,20), title(sprintf('Soluzione %d', s));
end
    
    

