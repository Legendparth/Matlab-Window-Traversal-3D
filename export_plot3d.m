% close all;

figure;
hold on;
grid on; 
axis equal;
xlabel('X (m)'); 
ylabel('Y (m)'); 
zlabel('Altitude (m)');
title('3D Dubins Path');
view(3);
plot(startPose(1, 1), startPose(1, 2), 'ro', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'HandleVisibility', 'off');
th = linspace(0, 2*pi, 200);
for i=1:num
    X = [wallTL(i, 1), wallTR(i, 1), wallBR(i, 1), wallBL(i, 1), wallTL(i, 1)];
    Y = [wallTL(i, 2), wallTR(i, 2), wallBR(i, 2), wallBL(i, 2), wallTL(i, 2)];
    Z = [wallTL(i, 3), wallTR(i, 3), wallBR(i, 3), wallBL(i, 3), wallTL(i, 3)];
    cc1(1:200) = startPose(i, 3); cc2(1:200) = goalPose(i, 3);
    plot3(C1(i, 1) + r*cos(th), C1(i, 2) + r*sin(th), cc1,'b--', 'HandleVisibility','off');
    plot3(C2(i, 1) + r*cos(th), C2(i, 2) + r*sin(th), cc2, 'r--', 'HandleVisibility','off');
    plot3(ref_posesXY(:,1,i), ref_posesXY(:,2,i), ref_posesZ(:,:,i)', ':', 'Color',[0.6 0.6 1], 'HandleVisibility','off', 'DisplayName', 'Dubins Path');
    plot3(X, Y, Z, 'r-', 'LineWidth', 3, 'HandleVisibility','off');
    plot3([goalPose(i, 1), midPoint(i, 1)],[goalPose(i, 2), midPoint(i, 2)],[goalPose(i, 3), midPoint(i, 3)], 'b-', 'LineWidth', 2,'DisplayName', 'Dubins Path st')
    plot3(startPose(i, 1), startPose(i, 2), startPose(i, 3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r','DisplayName', 'Start Position');
    plot3(goalPose(i, 1), goalPose(i, 2), goalPose(i, 3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g','DisplayName', 'Goal Position');
    plot3(A(i, 1), A(i, 2), A(i, 3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b','DisplayName', 'Tangent Points (A)');
    plot3(B(i, 1), B(i, 2), B(i, 3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b','DisplayName', 'Tangent Points (B)');
    plot3(out.x_path, out.y_path, out.z_path, 'k--', 'LineWidth', 2);
    arrow_length = 15;
    u_start = arrow_length * cos(startPose(i, 4));
    v_start = arrow_length * sin(startPose(i, 4));
    w_start = 0; 
    u_goal = arrow_length * cos(goalPose(i, 4));
    v_goal = arrow_length * sin(goalPose(i, 4));
    w_goal = 0;
    quiver3(startPose(i, 1), startPose(i, 2), startPose(i, 3), u_start, v_start, w_start, ...
        'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 1.5, 'AutoScale', 'off', 'DisplayName', 'Start');
    
    quiver3(goalPose(i, 1), goalPose(i, 2), goalPose(i, 3), u_goal, v_goal, w_goal, ...
        'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 1.5, 'AutoScale', 'off', 'DisplayName','End');
end

legend('Location', 'best', 'FontSize', 10);
hold off;