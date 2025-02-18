close all

vidFile = 'exampleVideo2.mp4';  % Name of the video file
v = VideoWriter(vidFile, 'MPEG-4');      % Create a video writer object
v.FrameRate = 10
open(v);                        % Open the video file for writing

error = sqrt(x_all(1,:).^2 + x_all(2,:).^2) - r;

figure(1);
hold on;
xlabel('time');
ylabel('distance to desired path');
grid on;
% axis equal
set(gca, 'FontSize', 16);
% legend('Location','southeast');
xlim([0, T(1, end)])
ylim([-1.8, 0.2])

for i = 1:10: switchpoint
    h_temp = plot(T(1,1:i), error(1,1:i), 'g-', 'linewidth',3, 'DisplayName', 'Pure pursuit control');
    drawnow;
    frame = getframe(gcf);
    writeVideo(v, frame);
    delete(h_temp);
end

for i = switchpoint + 1:10:size(x_all, 2)
    h_temp1 = plot(T(1,1:switchpoint), error(1,1:switchpoint), 'g-', 'linewidth',3, 'DisplayName', 'Pure pursuit control');
    h_temp2 = plot(T(1,switchpoint:i), error(1,switchpoint:i), 'b-','linewidth',3, 'DisplayName', 'Locally path-invariant control');
    drawnow;
    frame = getframe(gcf);
    writeVideo(v, frame);
    delete(h_temp1);
    delete(h_temp2);
end
close(v);

