function plotUKFResults(x_true, x_cor, z, P_cor, num_steps, delta_t)
    % Plot results of UKF
    
    % Convert time steps to actual time based on delta_t
    time = (0:num_steps-1) * delta_t;
    labels = {'X Position', 'Z Position', 'Velocity', 'Pitch Angle', 'Drag Coefficient'};
    
    % Plot state variables (true, estimated, observed)
    figure;
    
    for i = 1:5
        subplot(3, 2, i);
        plot(0:num_steps-1, x_true(i,:), '-', 'LineWidth', 2, 'Color', 'b', 'DisplayName', 'True State');
        hold on;
        plot(0:num_steps-1, x_cor(i,:), '--', 'LineWidth', 2, 'Color', 'r', 'DisplayName', 'Estimated State');
        hold on;
        if i <= 2
            plot(0:num_steps-1, z(i,:), '-', 'LineWidth', 2, 'Color', 'magenta', 'DisplayName', 'Observations');
        end
        xlabel('Time Step');
        ylabel(labels{i});
        legend;
        title([labels{i} ': True State vs Estimated State and Observations']);
    end

    % Plot trajectory
    subplot(3, 2, 6);
    plot(x_true(1,:), x_true(2,:), '-', 'LineWidth', 2, 'Color', 'b', 'DisplayName', 'True State');
    hold on;
    plot(x_cor(1,:), x_cor(2,:), '--', 'LineWidth', 2, 'Color', 'r', 'DisplayName', 'Estimated State');
    hold on;
    plot(z(1,:), z(2,:), '-', 'LineWidth', 2, 'Color', 'magenta', 'DisplayName', 'Observations');
    xlabel('X Position');
    ylabel('Z Position');
    legend;
    title('Trajectory: True State vs Estimated State and Observations');
    
    % Plot covariance matrices for all state variables
    figure;

    for i = 1:5
        subplot(5, 1, i);
        plot(time(2:end), squeeze(P_cor(i, i, 2:end)), '-d', 'LineWidth', 2, 'Color', 'k', 'DisplayName', ['P_{cor}(' num2str(i) ',' num2str(i) ')']);
        xlabel('Time (s)');
        ylabel(['P_{est}(' num2str(i) ',' num2str(i) ')']);
        legend;
        title(['Estimated State Covariance for ' labels{i}]);
    end
end
