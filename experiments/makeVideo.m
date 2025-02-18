close all

vidFile = 'exampleVideo.mp4';  % Name of the video file
v = VideoWriter(vidFile, 'MPEG-4');      % Create a video writer object
v.FrameRate = 10
open(v);                        % Open the video file for writing

figure(1);
hold on;
plot(xpath, ypath, 'k--', 'LineWidth',2, 'DisplayName', 'Motion plan');
lambda = -pi:0.01:pi;
plot(r*cos(lambda),r*sin(lambda), 'k-','linewidth',2, 'DisplayName', 'Desired path');
xlabel('x');
ylabel('y');
grid on;
axis equal
set(gca, 'FontSize', 16);
legend('Location','southeast');
% xlim([-3, 3])
% ylim([-3, 3])

for i = 1:10: switchpoint
    h_temp = plot(x_all(1,1:i), x_all(2,1:i), 'g-', 'linewidth',3, 'DisplayName', 'Pure pursuit control');
    drawnow;
    frame = getframe(gcf);
    writeVideo(v, frame);
    delete(h_temp);
end

for i = switchpoint + 1:10:size(x_all, 2)
    h_temp1 = plot(x_all(1,1:switchpoint), x_all(2,1:switchpoint), 'g-', 'linewidth',3, 'DisplayName', 'Pure pursuit control');
    h_temp2 = plot(x_all(1,switchpoint:i), x_all(2,switchpoint:i), 'b-','linewidth',3, 'DisplayName', 'Locally path-invariant control');
    drawnow;
    frame = getframe(gcf);
    writeVideo(v, frame);
    delete(h_temp1);
    delete(h_temp2);
end
close(v);

