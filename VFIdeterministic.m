close all

% Optimization problem:

% max beta * (c^(1-sigma))/(1-sigma))
% s.t. k'+ c = k^alpha + (1-delta)k
%      k_0 given

% Y = AK^alpha
% A = {Ah, Al}

% c = k^alpha + (1-delta)k - k'
% y = f(k) = c + i
% c = f(k) = Ak^alpha= k^alpha + (1-delta)k - k'
% c = Ak^alpha + (1-delta)k - k'

% state = k
% control = k'
% Dynamic programming problem:
% V (k) = ((Ak^alpha + (1-delta)k - k')^(1-sigma))/(1-sigma) + beta V(k',E[A'])


% transition matrix pi  = [pi^hh, 1-pi^ll, 1-pi^hh, pi^ll)
pi_hh = 0.977 
pi_ll = 0.926
trans = [pi_hh 1-pi_hh; 1-pi_ll pi_ll]


%%%% Set up parameters
alpha = 0.35;
beta = 0.99;
delta = 0.025;
sigma = 2;
interest = 0.04;
a_h = 1.1
a_l = 0.678

%%%% Set up discretized state space
k_min = 0;
k_max = 45;
num_k = 1000; % number of points in the grid for k

k = linspace(k_min, k_max, num_k);

k_mat = repmat(k', [1 num_k]); % this will be useful in a bit

%%%% Set up consumption and return function
% 1st dim(rows): k today, 2nd dim (cols): k' chosen for tomorrow
cons = k_mat .^ alpha + (1 - delta) * k_mat - k_mat'; 

c1 = a_h*k_mat .^ alpha + (1 - delta) * k_mat - k_mat'; 
c2 = a_l*k_mat .^ alpha + (1 - delta) * k_mat - k_mat'; 

ret = cons .^ (1 - sigma) / (1 - sigma); % return function

ret1 = c1.^ (1 - sigma) / (1 - sigma);
ret2 = c2.^ (1 - sigma) / (1 - sigma);


% negative consumption is not possible -> make it irrelevant by assigning
% it very large negative utility
ret(cons < 0) = -Inf;

ret1(cons1 < 0) = -Inf;
ret2(cons2 < 0) = -Inf;

%%%% Iteration
dis = 1; tol = 1e-06; % tolerance for stopping 
v_guess = zeros(1, num_k);
while dis > tol
    % compute the utility value for all possible combinations of k and k':
    value_mat = ret + beta * repmat(v_guess, [num_k 1]);
    
    value_mat1 = ret1 + beta * repmat(v_guess, [num_k 1]);
    value_mat2 = ret2 + beta * repmat(v_guess, [num_k 1]);
    
    % find the optimal k' for every k:
    [vfn, pol_indx] = max(value_mat, [], 2);
    vfn = vfn';
    
    % what is the distance between current guess and value function
    dis = max(abs(vfn - v_guess));
    
    % if distance is larger than tolerance, update current guess and
    % continue, otherwise exit the loop
    v_guess = vfn;
end

g = k(pol_indx); % policy function

plot(k,vfn)
figure
plot(k,g)


