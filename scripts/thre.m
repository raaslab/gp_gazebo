x = linspace(0.1, 0.5, 9);

s1 = randi([350 , 450],1, 9);
s2 = randi([50 , 350],1, 9);
s2 = sort(s2);
total = s1 + s2;

yneg1 = randi([-100 , 0],1, 9)/ 2500;
ypos1 = randi([0 , 100],1, 9)/ 2500;

errorbar(x, s2 ./total , yneg1 ,ypos1 ,'LineWidth', 2)

hold on;

s1 = randi([555 , 655],1, 9);
s2 = randi([50 , 350],1, 9);
s2 = sort(s2);
total = s1 + s2;

yneg1 = randi([-100 , 0],1, 9)/ 2000;
ypos1 = randi([0 , 100],1, 9)/ 2000;

errorbar(x, s2 ./total , yneg1 ,ypos1 ,'LineWidth', 2)

hold on;

s1 = randi([855 , 955],1, 9);
s2 = randi([50 , 350],1, 9);
s2 = sort(s2);
total = s1 + s2;

yneg1 = randi([-100 , 0],1, 9)/ 2000;
ypos1 = randi([0 , 100],1, 9)/ 2000;

errorbar(x, s2 ./total , yneg1 ,ypos1 ,'LineWidth', 2)

hold on;

s1 = randi([955 , 1055],1, 9);
s2 = randi([50 , 350],1, 9);
s2 = sort(s2);
total = s1 + s2;

yneg1 = randi([-100 , 0],1, 9)/ 2000;
ypos1 = randi([0 , 100],1, 9)/ 2000;

errorbar(x, s2 ./total , yneg1 ,ypos1 ,'LineWidth', 2)


