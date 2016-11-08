clc
close all
clear
% %
% Position10=importfile_P('Position10.txt');
% Position11=importfile_P('Position11.txt');
% Position12=importfile_P('Position12.txt');
% Position13=importfile_P('Position13.txt');
% Position14=importfile_P('Position14.txt');
% Position15=importfile_P('Position15.txt');
% 
% Position20=importfile_P('Position20.txt');
% Position21=importfile_P('Position21.txt');
% Position22=importfile_P('Position22.txt');
% Position23=importfile_P('Position23.txt');
% Position24=importfile_P('Position24.txt');
% Position25=importfile_P('Position25.txt');
% 
% Theta1=importfile_theta('Theta1.txt');
% Theta2=importfile_theta('Theta2.txt');
% 
% KUKA_1_Position=importfile('KUKA_1_Position.txt');
% KUKA_2_Position=importfile('KUKA_2_Position.txt');
% 
% 
% figure1 = figure;
% 
% % Create axes
% axes1 = axes('Parent',figure1);
% hold(axes1,'on');
% 
% % Create xlabel
% xlabel('X [m]');
% 
% % Create zlabel
% zlabel('Z [m]');
% 
% % Create ylabel
% ylabel('Y [m]');
% 
% box(axes1,'on');
% grid(axes1,'on');
% axis(axes1,'tight');
% % Set the remaining axes properties
% set(axes1,'DataAspectRatio',[1 1 1]);
% for i=1:size(Position10,1)
%     P1=[Position10(i,:);Position11(i,:);Position12(i,:);Position13(i,:);Position14(i,:);Position15(i,:)];
%     plot3(P1(:,1),P1(:,2),P1(:,3),'LineWidth',2, 'Color',[0.850980401039124 0.325490206480026 0.0980392172932625],'Marker','diamond');
%          pause(0.01)
%     hold on
% end
% for i=1:100:size(Position20,1)
%     P2=[Position20(i,:);Position21(i,:);Position22(i,:);Position23(i,:);Position24(i,:);Position25(i,:)];
%     plot3(P2(:,1),P2(:,2),P2(:,3),'LineWidth',2, 'Color',[0 0.447058826684952 0.74117648601532],'Marker','diamond');
%     pause(0.01)
%     hold on
% end

%%
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

Debug_position1_colided=importfile_debug('./Debug_positions/Debug_position1_colided.txt');
Debug_position2_colided=importfile_debug('./Debug_positions/Debug_position2_colided.txt');



for i=1:1000:size(Debug_position1_colided,1)
    P1=[Debug_position1_colided(i,1:3);Debug_position1_colided(i,4:6);Debug_position1_colided(i,7:9);Debug_position1_colided(i,10:12);Debug_position1_colided(i,13:15);Debug_position1_colided(i,16:18)];
    P2=[Debug_position2_colided(i,1:3);Debug_position2_colided(i,4:6);Debug_position2_colided(i,7:9);Debug_position2_colided(i,10:12);Debug_position2_colided(i,13:15);Debug_position2_colided(i,16:18)];
    plot3(P2(:,1),P2(:,2),P2(:,3),'LineWidth',2, 'Color',[0 0.447058826684952 0.74117648601532],'Marker','diamond');
    plot3(P1(:,1),P1(:,2),P1(:,3),'LineWidth',2, 'Color',[0.850980401039124 0.325490206480026 0.0980392172932625],'Marker','diamond');    
    pause(0.01)
end
 
%%
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

Debug_position1_neighbour=importfile_debug('./Debug_positions/Debug_position1_neighbour.txt');
Debug_position2_neighbour=importfile_debug('./Debug_positions/Debug_position2_neighbour.txt');

Neighbour=importfile_debug('./Complete_data/Neighbour_Complete.txt');

for i=1:1000:size(Debug_position1_neighbour,1)
    P1=[Debug_position1_neighbour(i,1:3);Debug_position1_neighbour(i,4:6);Debug_position1_neighbour(i,7:9);Debug_position1_neighbour(i,10:12);Debug_position1_neighbour(i,13:15);Debug_position1_neighbour(i,16:18)];
    P2=[Debug_position2_neighbour(i,1:3);Debug_position2_neighbour(i,4:6);Debug_position2_neighbour(i,7:9);Debug_position2_neighbour(i,10:12);Debug_position2_neighbour(i,13:15);Debug_position2_neighbour(i,16:18)];
    plot3(P2(:,1),P2(:,2),P2(:,3),'LineWidth',2, 'Color',[0 0.447058826684952 0.74117648601532],'Marker','diamond');
    plot3(P1(:,1),P1(:,2),P1(:,3),'LineWidth',2, 'Color',[0.850980401039124 0.325490206480026 0.0980392172932625],'Marker','diamond');    
    pause(0.01)
end

