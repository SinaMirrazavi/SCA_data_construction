clc
clear
close all
KUKA_Position1=importfile_END('KUKA_END_IIWA0.txt');


plot3(KUKA_Position1(:,1),KUKA_Position1(:,2),KUKA_Position1(:,3),'.')
