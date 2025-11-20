clc
clear 
close all

%% compose the optimization problem
import casadi.*

%Problem data
A_wall = 100;
A_flr = 10;
alpha = 0.5;
beta = 2;
gamma = 0.5;
delta = 2;

% Part 1
h = SX.sym('h');
w=  SX.sym('w'); 
d=  SX.sym('d'); 

% Part 2
obj = -h*d*w;

% Part 3
g = [2*(h*w+h*d);       
     w*d;  
     h/w;
     d/w];   

P = [];  

%Part 4
OPT_variables = [h;w;d];  
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);


opts = struct;
opts.ipopt.max_iter = 1000;
opts.ipopt.print_level = 0; 
opts.print_time = 0; 
opts.ipopt.acceptable_tol =1e-8; 
opts.ipopt.acceptable_obj_change_tol = 1e-6; 

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

%Part 5
args = struct;
args.lbx = 0;  
args.ubx = +inf;  
args.lbg = [-inf;-inf;alpha;gamma]; 
args.ubg = [A_wall;A_flr;beta;delta];  

args.p   =  [];  
args.x0  = [3;4;5]; 
sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

%Part 6
x_optimal = full(sol.x)   
f_optimal = -full(sol.f)   

