mat1 = load('reward_1_mfrl_rmax.txt');
mat2 = load('reward_one_sim_rmax.txt');
mat3 = load('reward_gp_mfrl.txt');
mat4 = load('reward_one_sim_gprmax.txt');

plot(linspace(25, 25 * size(mat1, 1), size(mat1, 1)), mat1);
hold on;

plot(linspace(25, 25 * size(mat2, 1), size(mat2, 1)), mat2);
hold on;


plot(linspace(25, 25 * size(mat3, 1), size(mat3, 1)), mat3);
hold on;

plot(linspace(25, 25 * size(mat4, 1), size(mat4, 1)), mat4);

