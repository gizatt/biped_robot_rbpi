%%
angles = [0 0 0 0 0 0];
[trans, jac] = get_forward_kin(angles);
trans{6}
jac

%%
% Some simple prototype inverse kin
pos_target = [3.0; 3.0; 3.0];
guess = rand([1,6])*90-45;
[trans, jac] = get_forward_kin(guess);
result = trans{end}*[0; 0; 0; 1];
err = (result(1:3) - pos_target(1:3))
olderr = err;
delta_err = [Inf; Inf; Inf]

%%
iters = 0;
while (norm(err) > 0.01 && norm(delta_err) ~= 0)
    guess = guess - (0.1*jac.'*err).';
    [trans, jac] = get_forward_kin(guess);
    result = trans{end}*[0; 0; 0; 1];
    err = (result(1:3) - pos_target(1:3));
    delta_err = err - olderr;
    olderr = err;
    iters = iters + 1
end

err
iters

%%
% Visualize;
close all;
figure;
hold off; hold on;
pts_all = {[], [], [], [], []};
for i=1:length(trans)-1
   for t=0:0.05:1
       pts_all{i} = [pts_all{i}; t*(trans{i}*[0;0;0;1]).' + (1-t)*(trans{i+1}*[0;0;0;1]).'];
   end
   scatter3(pts_all{i}(:, 1), pts_all{i}(:, 2), pts_all{i}(:, 3));
end
%axis([-5 5 -5 5 -1 9])

%% Visualize workspace:
% limits (going aroudn from left ankle)
joints_limits_deg = [-45, 45; -90 90; -60 60; -60 60; -90 90; -45 45];
joints_limits = joints_limits_deg * pi / 180;

%figure;
%hold off; hold on;
wk_steps = [0.3; 1.0; 1.0; 1.0; 1.0];
wk_points_total = 0;
for a = joints_limits(1, 1):wk_steps(1):joints_limits(1, 2)
    for b = joints_limits(2, 1):wk_steps(2):joints_limits(2, 2)
        for c = joints_limits(3, 1):wk_steps(3):joints_limits(3, 2)
            for d = joints_limits(4, 1):wk_steps(4):joints_limits(4, 2)
                for e = joints_limits(5, 1):wk_steps(5):joints_limits(5, 2)
                    wk_points_total = wk_points_total + 1;
                end
            end
        end
    end
end
i = 1;
wk_pts = zeros(wk_points_total, 4);
for a = joints_limits(1, 1):wk_steps(1):joints_limits(1, 2)
    for b = joints_limits(2, 1):wk_steps(2):joints_limits(2, 2)
        for c = joints_limits(3, 1):wk_steps(3):joints_limits(3, 2)
            for d = joints_limits(4, 1):wk_steps(4):joints_limits(4, 2)
                for e = joints_limits(5, 1):wk_steps(5):joints_limits(5, 2)
                    trans = get_forward_kin([a b c d e 0]*180/pi);
                    wk_pts(i, :) = trans{end}*[0;0;0;1];
                    i = i + 1;
                    if mod(i, 1000)==0
                        fprintf('%d/%d\n', i, wk_points_total);
                    end
                end
            end
        end
    end
end
scatter3(wk_pts(:, 1), wk_pts(:, 2), wk_pts(:, 3));
k = delaunay(wk_pts(:, 1), wk_pts(:, 2), wk_pts(:, 3));
%trimesh(k, wk_pts(:, 1), wk_pts(:, 2), wk_pts(:, 3))

      