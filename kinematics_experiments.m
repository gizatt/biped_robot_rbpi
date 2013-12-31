ar = [0; 0; 3]
al = [0; 0; 3]

kr = [0; 0; 2.4]
kl = [0; 0; 2.4]

hr = [-1.5; 0; 1]
hl = [1.5; 0; 1]

ar_theta = 0
al_theta = 0
kr_theta = 50
kl_theta = 50
hr_theta = 0
hl_theta = 0

% Stable stand on right foot
% ar_theta = 35
% al_theta = -35
% kr_theta = -50
% kl_theta = 0
% hr_theta = 50
% hl_theta = 0

% Convert to rads
ar_theta = ar_theta * pi / 180;
al_theta = al_theta * pi / 180;
kr_theta = kr_theta * pi / 180;
kl_theta = kl_theta * pi / 180;
hr_theta = hr_theta * pi / 180;
hl_theta = hl_theta * pi / 180;

ar_rot_up = [cos(ar_theta), 0, sin(ar_theta);
          0, 1, 0;
          -sin(ar_theta), 0, cos(ar_theta)];
ar_rot_down = [cos(-ar_theta), 0, sin(-ar_theta);
          0, 1, 0;
          -sin(-ar_theta), 0, cos(-ar_theta)];
      
al_rot_up = [cos(-al_theta), 0, sin(-al_theta);
          0, 1, 0;
          -sin(-al_theta), 0, cos(-al_theta)];
al_rot_down = [cos(al_theta), 0, sin(al_theta);
          0, 1, 0;
          -sin(al_theta), 0, cos(al_theta)];  
      
kr_rot_up = [1, 0, 0;
          0, cos(-kr_theta), -sin(-kr_theta);
          0, sin(-kr_theta), cos(-kr_theta)];
kr_rot_down = [1, 0, 0;
          0, cos(kr_theta), -sin(kr_theta);
          0, sin(kr_theta), cos(kr_theta)];
kl_rot_up = [1, 0, 0;
          0, cos(-kl_theta), -sin(-kl_theta);
          0, sin(-kl_theta), cos(-kl_theta)];
kl_rot_down = [1, 0, 0;
          0, cos(kl_theta), -sin(kl_theta);
          0, sin(kl_theta), cos(kl_theta)];  
      
hr_rot_up = [1, 0, 0;
          0, cos(-hr_theta), -sin(-hr_theta);
          0, sin(-hr_theta), cos(-hr_theta)];
hr_rot_down = [1, 0, 0;
          0, cos(hr_theta), -sin(hr_theta);
          0, sin(hr_theta), cos(hr_theta)];
hl_rot_up = [1, 0, 0;
          0, cos(-hl_theta), -sin(-hl_theta);
          0, sin(-hl_theta), cos(-hl_theta)];
hl_rot_down = [1, 0, 0;
          0, cos(hl_theta), -sin(hl_theta);
          0, sin(hl_theta), cos(hl_theta)];
      
% Passing through root needs to be flipped
% I don't really understand why it gets flipped
% exactly like this yet?
root_rot = [-1, 0, 0;
          0, -1, 0;
          0, 0, -1];
      
fprintf('In right ankle frame:')
r_ar_pos = [0;0;0]
fprintf('Right knee:')
r_kr_pos = ar_rot_up*ar
fprintf('Right Hip:')
r_hr_pos = ar_rot_up*(ar + kr_rot_up*kr)
fprintf('Root:')
r_root_pos = ar_rot_up*(ar + kr_rot_up*(kr + hr_rot_up*hr))
fprintf('Left hip:')
r_hl_pos = ar_rot_up*(ar + kr_rot_up*(kr + hr_rot_up*(hr+root_rot*hl)))
fprintf('Left knee:')
r_kl_pos = ar_rot_up*(ar + kr_rot_up*(kr + hr_rot_up*(hr+root_rot*(hl+hl_rot_down*kl))))
fprintf('Left ankle:')
r_al_pos = ar_rot_up*(ar + kr_rot_up*(kr + hr_rot_up*(hr+root_rot*(hl+hl_rot_down*(kl+kl_rot_down*al)))))

% Visualize
close all;
figure;
hold off; hold on;
r_ra2k = [];
r_rk2h = [];
r_rh2r = [];
r_la2k = [];
r_lk2h = [];
r_lh2r = [];

for t=0:0.01:1
    r_ra2k = [r_ra2k; t*r_kr_pos.' + (1-t)*r_ar_pos.'];
    r_rk2h = [r_rk2h; t*r_kr_pos.' + (1-t)*r_hr_pos.'];
    r_rh2r = [r_rh2r; t*r_hr_pos.' + (1-t)*r_root_pos.'];
    r_la2k = [r_la2k; t*r_kl_pos.' + (1-t)*r_al_pos.'];
    r_lk2h = [r_lk2h; t*r_kl_pos.' + (1-t)*r_hl_pos.'];
    r_lh2r = [r_lh2r; t*r_hl_pos.' + (1-t)*r_root_pos.'];
end
scatter3(r_ra2k(:, 1), r_ra2k(:, 2), r_ra2k(:, 3))
scatter3(r_rk2h(:, 1), r_rk2h(:, 2), r_rk2h(:, 3))
scatter3(r_rh2r(:, 1), r_rh2r(:, 2), r_rh2r(:, 3))
scatter3(r_la2k(:, 1), r_la2k(:, 2), r_la2k(:, 3))
scatter3(r_lk2h(:, 1), r_lk2h(:, 2), r_lk2h(:, 3))
scatter3(r_lh2r(:, 1), r_lh2r(:, 2), r_lh2r(:, 3))

axis([-5 5 -5 5 -1 9])