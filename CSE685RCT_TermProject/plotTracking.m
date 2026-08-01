% plotting
% by M. Amani E. 
% https://sites.google.com/view/maekuhi
function plotTracking(t, q, plotTitle)

qd = zeros(length(t),3);

for i = 1:length(t)
    qd(i,:) = desiredTrajectory(t(i))';
end

colors = lines(3);

hold on;

for j = 1:3
    plot(t, qd(:,j), "--", ...
        "Color", colors(j,:), "LineWidth", 1.8);

    plot(t, q(:,j), "-", ...
        "Color", colors(j,:), "LineWidth", 1.3);
end

grid on; box on;

xlabel("Time (s)");
ylabel("Joint position (rad)");
title(plotTitle);

legend("q_{d1}", "q_1", ...
    "q_{d2}", "q_2", ...
    "q_{d3}", "q_3", ...
    "Location", "best", ...
    "NumColumns", 2);

set(gca, "FontSize", 12);
end