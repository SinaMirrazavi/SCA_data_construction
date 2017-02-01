
/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Sina Mirrazavi
 * email:   sina.mirrazavi@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
clc
clear
close all
addpath('data')


File_name='KUKA_END_IIWA0.txt';
Max_num_GMM=20;


KUKA_Position=importfile_END(File_name);
plot3(KUKA_Position(:,1),KUKA_Position(:,2),KUKA_Position(:,3),'.')


options = statset('MaxIter',10000,'Display','final');

BIC = zeros(1,Max_num_GMM);
GMModels = cell(1,Max_num_GMM);

for i=1:Max_num_GMM
    GMModels{i} = fitgmdist(KUKA_Position,i,'RegularizationValue',0.001,'Replicates',5,'Options',options);
    Log(i)= GMModels{i}.NegativeLoglikelihood;
end

[minBIC,numComponents] = min(BIC);

BestModel = GMModels{numComponents};

probability=BestModel.pdf(KUKA_Position);

handle=sort(probability(:));
Threshold = handle(ceil(size(handle,1)*0.01));


save('IIWA_workspace_Model')


delete IIWA_workspace_Model_Threshold.txt
save('IIWA_workspace_Model_Threshold.txt', 'Threshold', '-ASCII', '-append');

prior=BestModel.ComponentProportion;
delete IIWA_workspace_Model_prior.txt
save('IIWA_workspace_Model_prior.txt', 'prior', '-ASCII', '-append');

mu=transpose(BestModel.mu);
delete IIWA_workspace_Model_mu.txt
save('IIWA_workspace_Model_mu.txt', 'mu', '-ASCII', '-append');

delete IIWA_workspace_Model_Sigma.txt
for i=1:BestModel.NumComponents
    Sigma=transpose(BestModel.Sigma(:,:,i));
    save('IIWA_workspace_Model_Sigma.txt', 'Sigma', '-ASCII', '-append');
end


