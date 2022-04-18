function [mu,Sigma] = imu_motion_model(mu_prev,Sigma_prev,u,M,dt)
  global n m k
  
  if ~min(size(mu_prev) == [n 1] & size(Sigma_prev) == [n n] & size(u) == [m 1] & size(M) == [m m])
    error('Expected inputs of size 5x1, 5x5, 3x1, and 3x3 respectively')
  end
  
  x_prev = mu_prev(1);
  y_prev = mu_prev(2);
  theta_prev = mu_prev(3);
  vx_prev = mu_prev(4);
  vy_prev = mu_prev(5);
  w_prev = mu_prev(6);
  
  ax = u(1);
  ay = u(2);
  wz = u(3);
  
%  A = speye(n);
%  A(1:2,4:5) = dt;
%  mu = A*mu_prev;
%  
%  B = dt*[0.5*dt*cos(theta_prev),-0.5*dt*sin(theta_prev), 0;
%          0.5*dt*sin(theta_prev), 0.5*dt*cos(theta_prev), 0;
%          0,                      0,                      1;
%          cos(theta_prev),       -sin(theta_prev),        0;
%          sin(theta_prev),        cos(theta_prev),        0];
%          
%  mu += B*u;
  
  x = x_prev + vx_prev*dt + 0.5*dt^2*(ax*cos(theta_prev)-ay*sin(theta_prev));
  y = y_prev + vy_prev*dt + 0.5*dt^2*(ax*sin(theta_prev)+ay*cos(theta_prev));
  theta = theta_prev + dt*0.5*(wz+w_prev);
  theta = mod(theta + pi,2*pi)-pi;
  vx = vx_prev + dt*(ax*cos(theta_prev)-ay*sin(theta_prev));
  vy = vy_prev + dt*(ax*sin(theta_prev)+ay*cos(theta_prev));
  w = (wz+w_prev)/2;
  
  mu = [x;y;theta;vx;vy;wz];
  
  % Jacobian of mu wrt mu_prev
  G = [1, 0,-0.5*dt^2*(ax*sin(theta_prev)+ay*cos(theta_prev)), dt,  0,  0;
       0, 1, 0.5*dt^2*(ax*cos(theta_prev)-ay*sin(theta_prev)),  0, dt,  0;
       0, 0, 1,                                                 0,  0, dt/2;
       0, 0,-dt*(ax*sin(theta_prev)+ay*cos(theta_prev)),        1,  0,  0;
       0, 0, dt*(ax*cos(theta_prev)-ay*sin(theta_prev)),        0,  1,  0;
       0, 0, 0,                                                 0,  0, 1/2];
  
  % Derivative of mu wrt u
  V = [0.5*dt^2*cos(theta_prev), -0.5*dt^2*sin(theta_prev), 0;
       0.5*dt^2*sin(theta_prev),  0.5*dt^2*cos(theta_prev), 0;
       0,                         0,                       dt/2;
       dt*cos(theta_prev),       -dt*sin(theta_prev),       0;
       dt*sin(theta_prev),        dt*cos(theta_prev),       0;
       0,                         0,                       0.5];
       
  
  Sigma = G*Sigma_prev*G' + V*M*V';
end