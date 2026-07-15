function robot = IRB1600_RBT_Generation()

% DH Parameters
a12 = 0.1515;
a23 = 0.72;
a34 = 0.2434;
a56 = 0.001;

d12 = 0.362;
d45 = 0.3548;
d67 = 0.115;

% Scaling Matrix
S = [0.01   0     0    0;
       0   0.01   0    0;
       0     0   0.01  0;
       0     0     0    1];

robot = rigidBodyTree("DataFormat","row","MaxNumBodies",14);
robot.Gravity = [0 0 -9.81];

%% BASE (FIXED)
baseBody = rigidBody("base_link");
baseBody.Mass = 50.029;
baseBody.CenterOfMass = [-0.103, 0.002, -0.066];
baseBody.Inertia = [0.876, 1.49, 2.216, -0.002, 0.06, 0.002];

T_mesh0 = S;
addVisual(baseBody, "Mesh", "link_0_base.stl", T_mesh0);

baseJoint = rigidBodyJoint("base_fix", "fixed");
setFixedTransform(baseJoint, trvec2tform([0 0 0])); 
baseBody.Joint = baseJoint;
addBody(robot, baseBody, robot.BaseName);

%% JOINT 1
body1 = rigidBody("body1");
body1.Mass = 70.568;
body1.CenterOfMass = [0.051, -0.009, 0.214];
body1.Inertia = [1.479, 2.435, 1.744, 0.157, -0.466, 0.11]; 

T_mesh1 = S;
addVisual(body1, "Mesh", "link_1_turret.stl", T_mesh1);

joint1 = rigidBodyJoint("joint1","revolute");
joint1.PositionLimits = [-2.62, 2.62];
joint1.JointAxis = [0 0 1];
setFixedTransform(joint1, eye(4));
body1.Joint = joint1;
addBody(robot, body1, "base_link");

% Fixed Transform
fix1 = rigidBody("fix1");
jfix1 = rigidBodyJoint("fixj1","fixed");
Tfix1 = trvec2tform([0 0 d12]) * ...
        trvec2tform([a12 0 0]) * ...
        axang2tform([0 0 1 -pi/2]) * ...
        axang2tform([0 1 0 -pi/2]);
setFixedTransform(jfix1, Tfix1);
fix1.Joint = jfix1;
addBody(robot, fix1, "body1");

%% JOINT 2
body2 = rigidBody("body2");
body2.Mass = 60.748;
body2.CenterOfMass = [0.00, -0.166, 0.318];
body2.Inertia = [4.105, 4.121, 0.186, -0.19, -0.005, 0.00];

T_mesh2 = S;
addVisual(body2, "Mesh", "link_2_lower_arm.stl", T_mesh2);

joint2 = rigidBodyJoint("joint2","revolute");
joint2.PositionLimits = [-1.1, 1.92];
joint2.JointAxis = [0 0 1];
setFixedTransform(joint2, eye(4));
body2.Joint = joint2;
addBody(robot, body2, "fix1");

% Fixed Transform
fix2 = rigidBody("fix2");
jfix2 = rigidBodyJoint("fixj2","fixed");
Tfix2 = trvec2tform([a23 0 0]) * ...
        axang2tform([0 0 1 pi/2]);
setFixedTransform(jfix2, Tfix2);
fix2.Joint = jfix2;
addBody(robot, fix2, "body2");

%% JOINT 3
body3 = rigidBody("body3");
body3.Mass = 50.813;
body3.CenterOfMass = [0.014, 0.022, -0.013];
body3.Inertia = [0.369, 0.761, 0.758, 0.027, -0.003, 0.022];

T_mesh3 = S;
addVisual(body3, "Mesh", "link_3_4_forearm.stl", T_mesh3);

joint3 = rigidBodyJoint("joint3","revolute");
joint3.PositionLimits = [-2.62, 2.62];
joint3.JointAxis = [0 0 1];
setFixedTransform(joint3, eye(4));
body3.Joint = joint3;
addBody(robot, body3, "fix2");

% Fixed Transform
fix3 = rigidBody("fix3");
jfix3 = rigidBodyJoint("fixj3","fixed");
Tfix3 = trvec2tform([a34 0 0]);
setFixedTransform(jfix3, Tfix3);
fix3.Joint = jfix3;
addBody(robot, fix3, "body3");

% Fixed Block 
fix4a = rigidBody("fix4a");
jfix4a = rigidBodyJoint("fixj4a","fixed");
Tfix4a = axang2tform([0 0 1 pi/2]) * ...
         axang2tform([1 0 0 pi/2]);
setFixedTransform(jfix4a, Tfix4a);
fix4a.Joint = jfix4a;
addBody(robot, fix4a, "fix3");

%% JOINT 4
body4 = rigidBody("body4");
body4.Mass = 19.37;
body4.CenterOfMass = [0.223, 0, 0];
body4.Inertia = [0.033, 0.237, 0.242, 1.611e-05, 0, 0.002];

T_mesh4 = axang2tform([1 0 0 -pi/2]) * S;
addVisual(body4, "Mesh", "link_5_6_wrist.stl", T_mesh4);

joint4 = rigidBodyJoint("joint4","revolute");
joint4.PositionLimits = [-3.14, 3.14];
joint4.JointAxis = [0 0 1];
setFixedTransform(joint4, eye(4));
body4.Joint = joint4;
addBody(robot, body4, "fix4a");

% Fixed Transform
fix4 = rigidBody("fix4");
jfix4 = rigidBodyJoint("fixj4","fixed");
Tfix4 = trvec2tform([0 0 d45]) * ...
        axang2tform([1 0 0 -pi/2]);
setFixedTransform(jfix4, Tfix4);
fix4.Joint = jfix4;
addBody(robot, fix4, "body4");

%% JOINT 5 (Wrist)
body5 = rigidBody("body5");
body5.Mass = 1;
body5.CenterOfMass = [0, 0, 0.04];
body5.Inertia = [0.0021, 0.0021, 0.0009, 0, 0, 0];

joint5 = rigidBodyJoint("joint5","revolute");
joint5.PositionLimits = [-2, 2];
joint5.JointAxis = [0 0 1];
setFixedTransform(joint5, eye(4));
body5.Joint = joint5;
addBody(robot, body5, "fix4");

% Fixed Transform
fix5 = rigidBody("fix5");
jfix5 = rigidBodyJoint("fixj5","fixed");
Tfix5 = axang2tform([0 0 1 -pi/2]) * ...
        trvec2tform([a56 0 0]);
setFixedTransform(jfix5, Tfix5);
fix5.Joint = jfix5;
addBody(robot, fix5, "body5");

% Fixed Block 
fix6a = rigidBody("fix6a");
jfix6a = rigidBodyJoint("fixj6a","fixed");
Tfix6a = axang2tform([0 0 1 pi/2]) * ...
         axang2tform([1 0 0 pi/2]);
setFixedTransform(jfix6a, Tfix6a);
fix6a.Joint = jfix6a;
addBody(robot, fix6a, "fix5");

%% JOINT 6
body6 = rigidBody("body6");
body6.Mass = 0.01;
body6.CenterOfMass = [0, 0, 0];
body6.Inertia = [1e-5, 1e-5, 1e-5, 0, 0, 0];

T_mesh7 = S;
addVisual(body6, "Mesh", "link_7_tool.stl", T_mesh7);

joint6 = rigidBodyJoint("joint6","revolute");
joint6.PositionLimits = [-3.14, 3.14];
joint6.JointAxis = [0 0 1];
setFixedTransform(joint6, eye(4));
body6.Joint = joint6;
addBody(robot, body6, "fix6a");

%% END EFFECTOR (tool)
tool = rigidBody("tool");
tool.Mass = 0;
tool.CenterOfMass = [0, 0, 0];
tool.Inertia = [0, 0, 0, 0, 0, 0];

jtool = rigidBodyJoint("toolFix","fixed");
Ttool = trvec2tform([0 0 d67]);
setFixedTransform(jtool, Ttool);
tool.Joint = jtool;


addBody(robot, tool, "body6");

% --- Robot Visualizzation ---
figure('Name', 'Rigid Body Tree, ABB-IRB1600');
show(robot, 'Visuals', 'on', 'Frames', 'on');

end
