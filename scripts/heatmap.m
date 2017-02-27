[X, Y] = meshgrid(-10: 1 :10);


mu1 = [-10 -10];
Sigma1 = .2 * [3 0; 0 3];

mu2 = [10 10];
Sigma2 = 16 * [3 0; 0 1];

mu3 = [9 -9];
Sigma3 = 12 * [3 0; 0 1];

% mu4 = [5 -5];
% Sigma4 = 6 * [0.5 0; 0 3];

% mu5 = [-5 5];
% Sigma5 = 6 * [0.5 0; 0 3];

% mu6 = [0 5];
% Sigma6 = 6 * [0.5 0; 0 3];

% mu7 = [0 -5];
% Sigma7 = 6 * [0.5 0; 0 3];

% F = mvnpdf([X(:) Y(:)],mu1,Sigma1) + mvnpdf([X(:) Y(:)],mu2,Sigma2) + mvnpdf([X(:) Y(:)],mu3,Sigma3); % ...
F = mvnpdf([X(:) Y(:)],mu3,Sigma3) + mvnpdf([X(:) Y(:)],mu2,Sigma2) + mvnpdf([X(:) Y(:)], mu1, Sigma1)./350; % ...

% + mvnpdf([X(:) Y(:)], mu4, Sigma4) + mvnpdf([X(:) Y(:)], mu5, Sigma5) ...
% + mvnpdf([X(:) Y(:)], mu6, Sigma6) + mvnpdf([X(:) Y(:)], mu7, Sigma7);

%F =  100 * mvnpdf([X(:) Y(:)],mu1,Sigma1);

F = reshape(F,length(X),length(Y));
F = 1000 * F; 
dlmwrite('variance2.txt',F,'delimiter','\t','precision',3)
size (F);
colormap('hot')

imagesc(F)
axis equal; 
% axis([-1,22,-1,22])
colorbar


% init = 104;
% x = [0.7, 1.3, 1.3, 0.7]
% y = [0, 0, init, init]
% fill(x, y, 'black')

% hold on;

% x = [1.7, 2.3, 2.3, 1.7]
% y = [init, init, init + 35, init + 35]
% fill(x, y, 'black')

% hold on;

% x = [0.7, 1.3, 1.3, 0.7]
% y = [init + 25, init + 25, init + 95, init + 95]
% fill(x, y, 'black')

% hold on;

% x = [1.7, 2.3, 2.3, 1.7]
% y = [init + 95, init + 95, init + 120, init + 120]
% fill(x, y, 'black')

% hold on;

% x = [0.7, 1.3, 1.3, 0.7]
% y = [init + 120, init + 120, init + 166, init + 166]
% fill(x, y, 'black')

% hold on;

% x = [1.7, 2.3, 2.3, 1.7]
% y = [init + 166, init + 166, init + 194, init + 194]
% fill(x, y, 'black')

% hold on;

% x = [0.7, 1.3, 1.3, 0.7]
% y = [init + 194, init + 194, init + 226, init + 226]
% fill(x, y, 'black')

% hold on;

% x = [1.7, 2.3, 2.3, 1.7]
% y = [init + 226, init + 226, init + 250, init + 250]
% fill(x, y, 'black')