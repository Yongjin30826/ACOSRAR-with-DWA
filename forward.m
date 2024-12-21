function x = forward(x, u)
% u = [vt; wt];当前时刻的速度、角速度
global dt;

F = [1 0 0 1 0 0;
    0 1 0 0 1 0;
    0 0 1 0 0 1;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];

B = [0 0 0;
     0 0 0;
     0 0 0;
     1 0 0;
     0 1 0;
     0 0 1];
x= F*x+B*u*dt;
