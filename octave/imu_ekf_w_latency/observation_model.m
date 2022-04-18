function [mu,Sigma] = observation_model(mu_prev,Sigma_prev,z,Q,u)
  global n m k latency
  
  if ~min(size(mu_prev) == [n 1] & size(Sigma_prev) == [n n] & size(z) == [k 1] & size(Q) == [k k])
    error('Expected inputs of size 5x1, 5x5, 3x1, and 3x3 respectively')
  end
  
%  x_prev = mu_prev(1);
%  y_prev = mu_prev(2);
%  theta_prev = mu_prev(3);
%  vx_prev = mu_prev(4);
%  vy_prev = mu_prev(5);
%  
%  x_z = z(1);
%  y_z = z(2);
%  theta_z = z(3);

  if abs(z(3) - mu_prev(3)) > abs(z(3) + 2*pi - mu_prev(3))
    z(3) = z(3) + 2*pi;
  elseif abs(z(3) - mu_prev(3)) > abs(z(3) - 2*pi - mu_prev(3))
    z(3) = z(3) - 2*pi;
  end
  
  % Camera latency
  d = latency;
  C = [1,0,0,-d, 0, 0;
       0,1,0, 0,-d, 0;
       0,0,1, 0, 0,-d];
       
%  %no latency
%  C = [1,0,0, 0, 0, 0;
%       0,1,0, 0, 0, 0;
%       0,0,1, 0, 0, 0];
  
  K = Sigma_prev*C'*inv(C*Sigma_prev*C' + Q);
  mu = mu_prev+K*(z-C*mu_prev);
  Sigma = (eye(n)-K*C)*Sigma_prev;
end