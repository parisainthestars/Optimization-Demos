clc
clear 
close all
%% Plot Setting
set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)
line_width = 1.5;
fontsize_labels = 14;

%% compose the optimization problem
import casadi.*
T = 0.2; % sampling time [s]
N = 30;  % prediction horizon

v_max = 0.6; v_min = -v_max;

omega_max = pi/4; omega_min = -omega_max;

x_max = 2; x_min = -x_max;

% Part 1
x = SX.sym('x');
y = SX.sym('y'); 
theta = SX.sym('theta');
states = [x;y;theta]; 
n_states = length(states);

v = SX.sym('v');
omega = SX.sym('omega');
controls = [v;omega];
n_controls = length(controls);

rhs = [v*cos(theta);
       v*sin(theta);
       omega]; % system r.h.s

f = Function('f',{states,controls},{rhs}); 
% nonlinear mapping function f(x,u)

U = SX.sym('U',n_controls,N); 
% Decision variables (controls)

X = SX.sym('X',n_states,(N+1));
% A Matrix that represents the states over the optimization problem.

P = SX.sym('P',n_states + n_states);
%P which include the initial and the reference state of the robot.

% compute solution symbolically
X(:,1) = P(1:n_states); % initial state
for k = 1:N
    st = X(:,k);  
    con = U(:,k);
    f_value  = f(st,con);
    st_next  = st+ (T*f_value);
    X(:,k+1) = st_next;
end

% this function to get the optimal trajectory knowing the optimal solution
ff=Function('ff',{U,P},{X});


% Part 2
obj = 0; % Objective function
Q = zeros(n_states,n_states); 
Q(1,1) = 1; Q(2,2) = 5; Q(3,3) = 0.1; % weighing matrices (states)
R = zeros(n_controls,n_controls); 
R(1,1) = 0.5; R(2,2) = 0.05; % weighing matrices (controls)

% compute objective
for k=1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(n_states+1:n_states*2))'*Q*(st-P(n_states+1:n_states*2)) ...
        + con'*R*con; % calculate obj
end
%obj = obj + (X(:,N+1)-P(n_states+1:n_states*2))'*Q*(X(:,N+1)-P(n_states+1:n_states*2));

% Part 3
g = [];  % constraints vector

% compute constraints
for k = 1:N+1   % box constraints due to the map margins
    g = [g ; X(1,k)];   %state x
    g = [g ; X(2,k)];   %state y
end

% Part 4
% make the decision variables one column vector
OPT_variables = reshape(U,n_controls*N,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

%Part 5
args = struct;

% input constraints
args.lbx(1:n_controls:n_controls*N-1,1) = v_min; 
args.lbx(2:n_controls:n_controls*N,1)   = omega_min;
args.ubx(1:n_controls:n_controls*N-1,1) = v_max; 
args.ubx(2:n_controls:n_controls*N,1)   = omega_max;

% inequality constraints (state constraints)
args.lbg = x_min;  % lower bound of the states x and y
args.ubg = x_max;   % upper bound of the states x and y 


%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SETTING UP

% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
x0 = [0 ; 0 ; 0];    % initial condition.
xs = [1.5 ; 1.5 ; 0]; % Reference posture.


u0 = zeros(N,n_controls);  % two control inputs 

args.p   = [x0;xs]; % set the values of the parameters vector
args.x0 = reshape(u0',n_controls*N,1); % initial value of the optimization variables

tic
sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
toc

u = reshape(full(sol.x)',n_controls,N);
xx = full(ff(u,args.p)); % compute OPTIMAL solution TRAJECTORY

%% Plot Results
x_r_1 = [];
y_r_1 = [];
r = 0.15;  % obstacle radius
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);
figure()
% Animate the robot motion
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

for k = 1:size(xx,2)
    h_t = 0.14; w_t=0.09; % triangle parameters

    x1 = xs(1); y1 = xs(2); th1 = xs(3);
    x1_tra = xx(1,k); y1_tra = xx(2,k);
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); % plot reference state
    hold on;
    x1 = xx(1,k,1); y1 = xx(2,k,1); th1 = xx(3,k,1);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    
    fill(x1_tri, y1_tri, 'r'); % plot robot position
    
    hold off
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([-0.2 1.8 -0.2 1.8])
    pause(0.1)
    box on;
    grid on
    drawnow
    % for video generation
    F(k) = getframe(gcf); % to get the current frame
end
close(gcf)
% 
% 
t=0:T:(N-1)*T;

upper_v=repmat(v_max,numel(t),1);
lower_v=repmat(v_min,numel(t),1);

upper_omega=repmat(omega_max,numel(t),1);
lower_omega=repmat(omega_min,numel(t),1);

figure
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 0.9]);

subplot(211)
plot(t,u(1,:),'color','blue','linewidth',1.5)
hold on
plot(t,upper_v,t,lower_v,'color','red','linewidth',1.5)
grid on
ylabel('v')
axis([0 t(end) v_min-0.1 +v_max+0.1])


subplot(212)
plot(t,u(2,:),'color','blue','linewidth',1.5)
hold on
plot(t,upper_omega,t,lower_omega,'color','red','linewidth',1.5)
grid on
ylabel('\omega')
xlabel('Time')
sgtitle('Input Controls')
axis([0 t(end) omega_min-0.2 +omega_max+0.2])

