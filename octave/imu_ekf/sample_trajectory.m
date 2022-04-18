function [X,U,Z] = sample_trajectory(t,M,Q)
  n = 5;
  m = 3;
  k = 3;
  
  r = 1;
  w = 0.5;
  
%% Lemniscate
  x = r.*cos(w.*t)./(1+sin(w.*t).^2);
  y = r.*sin(w.*t).*cos(w.*t)./(1+sin(w.*t).^2);
  theta = w.*t;
  theta = mod(theta+pi,2*pi)-pi;
  
%  vx = -r.*w.*sin(w.*t).*(sin(w.*t).^2 + 2*cos(w.*t).^2 + 1)./(sin(w.*t).^2 + 1).^2;
%  vy = -(r.*((w.*sin(t).^2+w).*sin(w.*t).^2+2.*cos(t).*sin(t).*cos(w.*t).*sin(w.*t)+(-w.*sin(t).^2-w).*cos(w.*t).^2))./(sin(t).^2+1).^2;
  vx = -(r.*w.*sin(w.*t).*(sin(w.*t).^2+2.*cos(w.*t).^2+1))./(sin(w.*t).^2+1).^2;
  vy = -(r.*w.*(sin(w.*t).^4+(cos(w.*t).^2+1).*sin(w.*t).^2-cos(w.*t).^2))./(sin(w.*t).^2+1).^2;
  
%  ax1 = (r.*((4.*w.*cos(t).*sin(t).^3+4.*w.*cos(t).*sin(t)).*sin(w.*t)+((2-w.^2).*sin(t).^4+(6.*cos(t).^2-2.*w.^2+2).*sin(t).^2-2.*cos(t).^2-w.^2).*cos(w.*t)))./(sin(t).^2+1).^3;
%  ay1 = (2.*r.*((2.*w.*cos(t).*sin(t).^3+2.*w.*cos(t).*sin(t)).*sin(w.*t).^2+((1-2.*w.^2).*sin(t).^4+(3.*cos(t).^2-4.*w.^2+1).*sin(t).^2-cos(t).^2-2.*w.^2).*cos(w.*t).*sin(w.*t)+(-2.*w.*cos(t).*sin(t).^3-2.*w.*cos(t).*sin(t)).*cos(w.*t).^2))./(sin(t).^2+1).^3;
  ax1 = (r.*w.^2.*cos(w.*t).*(5.*sin(w.*t).^4+(6.*cos(w.*t).^2+4).*sin(w.*t).^2-2.*cos(w.*t).^2-1))./(sin(w.*t).^2+1).^3;
  ay1 = (2.*r.*w.^2.*cos(w.*t).*sin(w.*t).*(sin(w.*t).^4+(cos(w.*t).^2-1).*sin(w.*t).^2-3.*cos(w.*t).^2-2))./(sin(w.*t).^2+1).^3;
  
  ax = ax1.*cos(theta)+ay1.*sin(theta);
  ay =-ax1.*sin(theta)+ay1.*cos(theta);
  
  wz = w*ones(1,length(t));%theta./t;

%% Circle
%  x = r*cos(w*t);
%  y = r*sin(w*t);
%  theta = w*t;
%  vx = -r*w*sin(w*t);
%  vy =  r*w*cos(w*t);
%  
%  wz = theta/t;
%  ax = r*wz.^2;
%  ay = 0*wz;
  
  X = [x;y;theta;vx;vy];
  U = [ax;ay;wz] + mvnrnd(zeros(m,1),M,length(t))';
  Z = [x;y;theta] + mvnrnd(zeros(k,1),Q,length(t))';
%  Z(3,:) = mod(Z(3,:) + pi, 2*pi) - pi;
  
end  