clc
close all
clear
% %
Position10=importfile_P('Position00.txt');
Position11=importfile_P('Position01.txt');
Position12=importfile_P('Position02.txt');
Position13=importfile_P('Position03.txt');
Position14=importfile_P('Position04.txt');
Position15=importfile_P('Position05.txt');
Position16=importfile_P('Position06.txt');

Position20=importfile_P('Position10.txt');
Position21=importfile_P('Position11.txt');
Position22=importfile_P('Position12.txt');
Position23=importfile_P('Position13.txt');
Position24=importfile_P('Position14.txt');
Position25=importfile_P('Position15.txt');
Position26=importfile_P('Position16.txt');

Theta1=importfile_theta('Theta0.txt');
Theta2=importfile_theta('Theta1.txt');

% KUKA_1_Position=importfile('KUKA_1_Position.txt');
% KUKA_2_Position=importfile('KUKA_2_Position.txt');


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

box(axes1,'on');
grid(axes1,'on');
axis(axes1,'tight');
% Set the remaining axes properties
set(axes1,'DataAspectRatio',[1 1 1]);
for i=1:10:size(Position10,1)
    P1=[Position10(i,:);Position11(i,:);Position12(i,:);Position13(i,:);Position14(i,:);Position15(i,:);Position16(i,:)];
    plot3(P1(:,1),P1(:,2),P1(:,3),'LineWidth',2, 'Color',[0.850980401039124 0.325490206480026 0.0980392172932625],'Marker','diamond');
         pause(0.01)
    hold on
end
for i=1:10:size(Position20,1)
    P2=[Position20(i,:);Position21(i,:);Position22(i,:);Position23(i,:);Position24(i,:);Position25(i,:);Position26(i,:)];
    plot3(P2(:,1),P2(:,2),P2(:,3),'LineWidth',2, 'Color',[0 0.447058826684952 0.74117648601532],'Marker','diamond');
    pause(0.01)
    hold on
end

%%
 close all
 figure1 = figure;

% Create axes
 axes1 = axes('Parent',figure1);
 hold(axes1,'on');
hold on

% Create xlabel
xlabel('X [m]');

% Create zlabel
zlabel('Z [m]');

% Create ylabel
ylabel('Y [m]');

box on
grid on
axis on

 box(axes1,'on');
 grid(axes1,'on');
 axis(axes1,'tight');
% % Set the remaining axes properties
 set(axes1,'DataAspectRatio',[1 1 1]);

Debug_position1_colided=importfileCollision('position1_colided01.txt');
Debug_position2_colided=importfileCollision('position2_colided01.txt');



for i=1:100:size(Debug_position1_colided,1)
    P1=[Debug_position1_colided(i,1:3);Debug_position1_colided(i,4:6);Debug_position1_colided(i,7:9);Debug_position1_colided(i,10:12);Debug_position1_colided(i,13:15);Debug_position1_colided(i,16:18);Debug_position1_colided(i,19:21)];
    P2=[Debug_position2_colided(i,1:3);Debug_position2_colided(i,4:6);Debug_position2_colided(i,7:9);Debug_position2_colided(i,10:12);Debug_position2_colided(i,13:15);Debug_position2_colided(i,16:18);Debug_position2_colided(i,19:21)];
    h1=plot3(P2(:,1),P2(:,2),P2(:,3),'LineWidth',2, 'Color',[0 0.447058826684952 0.74117648601532],'Marker','diamond');
    h2=plot3(P1(:,1),P1(:,2),P1(:,3),'LineWidth',2, 'Color',[0.850980401039124 0.325490206480026 0.0980392172932625],'Marker','diamond');
    pause(1)
    delete(h1)
    delete(h2)
end

%%
close all
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

box(axes1,'on');
grid(axes1,'on');
axis(axes1,'tight');
% Set the remaining axes properties
set(axes1,'DataAspectRatio',[1 1 1]);

Debug_position1_neighbour=importfileCollision('position1_neighbour01.txt');
Debug_position2_neighbour=importfileCollision('position2_neighbour01.txt');


for i=1:1000:size(Debug_position1_neighbour,1)
    P1=[Debug_position1_neighbour(i,1:3);Debug_position1_neighbour(i,4:6);Debug_position1_neighbour(i,7:9);Debug_position1_neighbour(i,10:12);Debug_position1_neighbour(i,13:15);Debug_position1_neighbour(i,16:18);Debug_position1_neighbour(i,19:21)];
    P2=[Debug_position2_neighbour(i,1:3);Debug_position2_neighbour(i,4:6);Debug_position2_neighbour(i,7:9);Debug_position2_neighbour(i,10:12);Debug_position2_neighbour(i,13:15);Debug_position2_neighbour(i,16:18);Debug_position2_neighbour(i,19:21)];
    h1=plot3(P2(:,1),P2(:,2),P2(:,3),'LineWidth',2, 'Color',[0 0.447058826684952 0.74117648601532],'Marker','diamond');
    h2=plot3(P1(:,1),P1(:,2),P1(:,3),'LineWidth',2, 'Color',[0.850980401039124 0.325490206480026 0.0980392172932625],'Marker','diamond');
    pause(1)
    delete(h1)
    delete(h2)
end

