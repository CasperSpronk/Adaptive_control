figure
plot(Path.waypoints(:,2), Path.waypoints(:,1), '--r', 'LineWidth', 1.5)
hold on
plot(Path.waypoints(:,2), Path.waypoints(:,1), '*b', 'LineWidth', 1)
plot(SimOut.PlantData.xyh.Data(:,2), SimOut.PlantData.xyh.Data(:,1),'LineWidth', 1.5);
grid minor

for i=2:size(Path.waypoints,1)-1
    q_i_min_1 = (Path.waypoints(i,:)-Path.waypoints(i-1,:)) / norm(Path.waypoints(i,:)-Path.waypoints(i-1,:),2);
    q_i = (Path.waypoints(i+1,:)-Path.waypoints(i,:)) / norm(Path.waypoints(i+1,:)-Path.waypoints(i,:),2);
    rho = acos(-q_i_min_1*q_i');
    c(i,:) = Path.waypoints(i,:)-(Guid.Loiter.R/(sin(rho/2)))*(q_i_min_1 - q_i)/(norm(q_i_min_1- q_i));
    drawCircle([c(i,2) c(i,1) Guid.Loiter.R], '--', 'color', 'g', 'linewidth', 1.5);
end
plot(c(:,2),c(:,1), '*g', 'LineWidth', 1)
title('Bixler Path')
xlabel('East [m]'); ylabel('North [m]');
axis square

clear c rho q_i q_i_min_1 