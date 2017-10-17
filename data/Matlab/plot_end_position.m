clc
clear
close all
KUKA_Position1=importfile_END('Val_Arm_Position0.txt');
KUKA_Position2=importfile_END('Val_Arm_Position1.txt');


figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create xlabel
xlabel('X [m]');

% Create zlabel
zlabel('Z [m]');

% Create ylabel
ylabel('Y [m]');

view(axes1,[-37.5 30]);
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',24);

plot3(KUKA_Position1(:,1),KUKA_Position1(:,2),KUKA_Position1(:,3),'.')
hold on
plot3(KUKA_Position2(:,1),KUKA_Position2(:,2),KUKA_Position2(:,3),'.')



figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create xlabel
xlabel('X [m]');

% Create zlabel
zlabel('Z [m]');

% Create ylabel
ylabel('Y [m]');

view(axes1,[-37.5 30]);
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',24);

plot3(KUKA_Position1(:,4),KUKA_Position1(:,5),KUKA_Position1(:,6),'.')
hold on
plot3(KUKA_Position2(:,4),KUKA_Position2(:,5),KUKA_Position2(:,6),'.')



figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create xlabel
xlabel('X [m]');

% Create zlabel
zlabel('Z [m]');

% Create ylabel
ylabel('Y [m]');

view(axes1,[-37.5 30]);
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',24);

plot3(KUKA_Position1(:,7),KUKA_Position1(:,8),KUKA_Position1(:,9),'.')
hold on
plot3(KUKA_Position2(:,7),KUKA_Position2(:,8),KUKA_Position2(:,9),'.')


figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create xlabel
xlabel('X [m]');

% Create zlabel
zlabel('Z [m]');

% Create ylabel
ylabel('Y [m]');

view(axes1,[-37.5 30]);
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',24);

plot3(KUKA_Position1(:,10),KUKA_Position1(:,11),KUKA_Position1(:,12),'.')
hold on
plot3(KUKA_Position2(:,10),KUKA_Position2(:,11),KUKA_Position2(:,12),'.')


figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create xlabel
xlabel('X [m]');

% Create zlabel
zlabel('Z [m]');

% Create ylabel
ylabel('Y [m]');

view(axes1,[-37.5 30]);
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',24);

plot3(KUKA_Position1(:,13),KUKA_Position1(:,14),KUKA_Position1(:,15),'.')
hold on
plot3(KUKA_Position2(:,13),KUKA_Position2(:,14),KUKA_Position2(:,15),'.')


figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create xlabel
xlabel('X [m]');

% Create zlabel
zlabel('Z [m]');

% Create ylabel
ylabel('Y [m]');

view(axes1,[-37.5 30]);
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',24);

plot3(KUKA_Position1(:,16),KUKA_Position1(:,17),KUKA_Position1(:,18),'.')
hold on
plot3(KUKA_Position2(:,16),KUKA_Position2(:,17),KUKA_Position2(:,18),'.')


figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create xlabel
xlabel('X [m]');

% Create zlabel
zlabel('Z [m]');

% Create ylabel
ylabel('Y [m]');

view(axes1,[-37.5 30]);
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',24);

plot3(KUKA_Position1(:,19),KUKA_Position1(:,20),KUKA_Position1(:,21),'.')
hold on
plot3(KUKA_Position2(:,19),KUKA_Position2(:,20),KUKA_Position2(:,21),'.')
