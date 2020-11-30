%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

x = z(1:7, :);
dx = z(8:14, :);

subplot(2,2,1);
plot(t, x);
title("Position");
xlabel("time (s)");
ylabel("Angle (rad)");

subplot(2,2,2);
plot(t, dx);
title("Velocity");
xlabel("time (s)");
ylabel("Angular Vel (rad/s)");

subplot(2,2,3);
plot(t, u);
title("Input");
xlabel("time (s)");
ylabel("Torque (Nm)");

subplot(2,2,4);
exampleHelperVisualizeCollisionEnvironment(worldCollisionModel);
for i=1:length(t)
    robot.show(x(:,i), 'PreservePlot', false, 'Frames','off') ;
    drawnow;
    if i ~= 1
        pause(t(i)-t(i-1));
    end
end