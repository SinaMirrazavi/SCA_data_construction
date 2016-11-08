close all
clear
clc

% Position25=importfile('Position25.txt');
Position05=importfile('Position05.txt');
Position15=importfile('Position15.txt');
% plot3(Position25(:,1),Position25(:,2),Position25(:,3),'.');
hold on
plot3(Position15(:,1),Position15(:,2),Position15(:,3),'.');
hold on
plot3(Position05(:,1),Position05(:,2),Position05(:,3),'.');
hold on
% Create xlabel
xlabel('X');

% Create zlabel
zlabel('Z');

% Create ylabel
ylabel('Y');

view(axes1,[-54.3 38.8]);
grid(axes1,'on');
% Create legend
legend(axes1,'show');
