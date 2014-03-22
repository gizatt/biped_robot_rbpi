function [trans, jac] = get_forward_kin(angles)
%% Angles input in order around from left ankle

%% We'll call the left ankle the root of the system,
% with forward being +y, right being +x, and up being +z.
% These are relevant displacements being joint origins:
ankle_to_knee = [0;0;3];
knee_to_hip = [0;0;2.4];
hip_to_hip = [3;0;0];

% Angle states of each joint:
% #  Angles per joint are defined:
% #    Ankle: 0 degree is flat on ground. 90 is bottom of foot
% #    pointing outwards. -90 is bottom of foot pointing inwards.
% #    (Due to frame of foot, angle of 90 or -90 can't actually be
% #    achieved without self-collision.)
% #    Knee: 0 degree is a straight knee. -90 is foot behind knee.
% #    +90 is foot in front of knee.
% #    Hip: To maximize range of motion in both directions, we'll
% #    define 0 degree to be the servo being at a 45 degree angle
% #    with the control backpack / board facing up and backwards.
% #    +90 degrees is the bottom of the foot facing straight forward
% #    (assuming straight knee), -90 degree is the bottom of the foot
% #    facing straight backward. Again, due to self-collision
% #    these angles can't be achieved.
al_theta = angles(1);
kl_theta = angles(2);
hl_theta = angles(3);
hr_theta = angles(4);
kr_theta = angles(5);
ar_theta = angles(6);

% (Convert to rads)
ar_theta = ar_theta * pi / 180;
al_theta = al_theta * pi / 180;
kr_theta = kr_theta * pi / 180;
kl_theta = kl_theta * pi / 180;
hr_theta = hr_theta * pi / 180;
hl_theta = hl_theta * pi / 180;


% Rotations associated with each joint
% (in angle-axis so I can conveniently feed through to matrix)
% (but converted to rot matrices)
% Ankle: rotation is about y axis
al_axis = [0 -1 0 al_theta];
ar_axis = [0 1 0 ar_theta];
% Knee and hip: rotation is about x axis
kl_axis = [-1 0 0 kl_theta];
kr_axis = [1 0 0 kr_theta];
hl_axis = [-1 0 0 hl_theta];
hr_axis = [1 0 0 hr_theta];
axes = {al_axis; kl_axis; hl_axis; hr_axis; kr_axis; ar_axis};

% generate rotation mats
al_rot = vrrotvec2mat(al_axis);
ar_rot = vrrotvec2mat(ar_axis);
kl_rot = vrrotvec2mat(kl_axis);
kr_rot = vrrotvec2mat(kr_axis);
hl_rot = vrrotvec2mat(hl_axis);
hr_rot = vrrotvec2mat(hr_axis);

% Transforms around from left ankle frame...
R_al_kl = [al_rot al_rot*ankle_to_knee; 0 0 0 1];
R_kl_hl = [kl_rot kl_rot*knee_to_hip; 0 0 0 1];
R_hl_hr = [hl_rot hl_rot*hip_to_hip; 0 0 0 1];
R_hr_kr = [hr_rot -hr_rot*knee_to_hip; 0 0 0 1];
R_kr_ar = [kr_rot -kr_rot*ankle_to_knee; 0 0 0 1];
R_ar = [ar_rot [0;0;0]; 0 0 0 1];
l_transforms = {R_al_kl, R_kl_hl, R_hl_hr, R_hr_kr, R_kr_ar, R_ar};

% Generate transforms to each other joint
transform = eye(4);
trans = cell(1, length(l_transforms));
for i=1:length(l_transforms)
    trans{i} = transform;
    transform = transform*l_transforms{i};
end
trans{end} = transform;

% Generate Jacobian
% The ith column is the cross product of the axis of
% revolution and the vec from the joint to the end effector we care
% about.
% (see http://graphics.cs.cmu.edu/nsp/course/15-464/Fall09/handouts/IK.pdf)
jac = [];
end_pos = trans{end}*[0;0;0;1];

for i=1:length(trans)
    axis_pos = trans{i}*[0;0;0;1];
    axis_dir = trans{i}*[axes{i}(1:3).'; 1];
    jac = [jac cross(axis_dir(1:3), end_pos(1:3)-axis_pos(1:3))];
end

end