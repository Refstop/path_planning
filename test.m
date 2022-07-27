clc;clear all;close all;

% ls_temp = load('ls_temp.mat').ls_temp;
% C_ls = load('ls_temp.mat').C_ls;
% 
% plot(ls_temp(:,1), ls_temp(:,2), '.');
% grid on;
% 
% i=2;
% normalized_ls_temp = [ls_temp(:,1) - C_ls(i,1), ls_temp(:,2) - C_ls(i,2)];
% % plot(normalized_ls_temp(:,1), normalized_ls_temp(:,2), '.');
% % grid on;
% C = cov(normalized_ls_temp);
% [E,D] = eig(C);
% % hold on; drawArrow(C_ls(i,1)+[0, D(1,1)/2*E(1,1)], C_ls(i,2)+[0, D(1,1)/2*E(2,1)], "red");
% % hold on; drawArrow(C_ls(i,1)+[0, D(2,2)/2*E(1,2)], C_ls(i,2)+[0, D(2,2)/2*E(2,2)], "blue");
% hold on; drawArrow(C_ls(i,1)+[0, E(1,1)], C_ls(i,2)+[0, E(2,1)], "red");
% hold on; drawArrow(C_ls(i,1)+[0, E(1,2)], C_ls(i,2)+[0, E(2,2)], "blue");
% xlim(C_ls(i,1)+[-0.5 0.5]); ylim(C_ls(i,2)+[-0.3 0.3]); 

x = [-10:0.01:10];
y = x.^2 + 0.1*randn(1, length(x));
x = x + 0.1*randn(1, length(x));
norm_X = [x - mean(x); y - mean(y)];
plot(norm_X(1,:), norm_X(2,:), '.');
grid on;
C = cov(norm_X');
[E,D] = eig(C);
hold on; drawArrow([0, D(1,1)/2*E(1,1)], [0, D(1,1)/2*E(2,1)], "red");
hold on; drawArrow([0, D(2,2)/2*E(1,2)], [0, D(2,2)/2*E(2,2)], "blue");