pkg load statistics

global n m k

n = 5;
m = 3;
k = 3;

imu_update_rate = 100;
dt_imu = 1/imu_update_rate;

camera_fps = 1;

%filter_update_rate = lcm(imu_update_rate,camera_fps);

tmax = 20*pi;

t = 0:dt_imu:tmax;
N = length(t);

% IMU covariance
M = (diag([0.025,0.025,0.002])).^2;

% Vision covariance
Q = (diag([0.005,0.005,0.1])).^2;

[x,u,z] = sample_trajectory(t,M,Q);

mu = zeros(n,N);
Sigma = zeros(n,n,N);

%mu(1:3,1) = z(1);
mu(:,1)=[0;0;0;0;0];%x(:,1);

Sigma(:,:,1) = eye(n);
Sigma(1:3,1:3,1) = Q;
Sigma(:,:,1) = eye(n);



for i = 2:N
  
  [mubar,Sigmabar] = imu_motion_model(mu(:,i-1),Sigma(:,:,i-1),u(:,i),M,dt_imu);
  
  if (mod(i,floor(imu_update_rate/camera_fps)))
    [mu(:,i),Sigma(:,:,i)] = observation_model(mubar,Sigmabar,z(:,i),Q);
  else
    mu(:,i)=mubar;
    Sigma(:,:,i)=Sigmabar;
  end
  
end
clf


%subplot(2,1,1)
%plot(t,mu)
%legend(["x";"y";"theta";"vx";"vy"])

%
%subplot(2,1,2)
%plot(t,x)
%legend(["x";"y";"theta";"vx";"vy"])

subplot(2,1,1);

plot(z(1,[1 floor(imu_update_rate/camera_fps):floor(imu_update_rate/camera_fps):N]),z(2,[1 floor(imu_update_rate/camera_fps):floor(imu_update_rate/camera_fps):N]),'.')
hold on
plot(x(1,:),x(2,:),'r','LineWidth',2)
plot(mu(1,:),mu(2,:),'g','LineWidth',2)
axis equal

subplot(2,1,2)
%plot(t,[zeros(1,N);x-mu])%sqrt((mu(1,:)-x(1,:)).^2+(mu(2,:)-mu(2,:)).^2)])
%axis([0, tmax, -0.01, 0.01])
%legend(["x";"y";"theta";"vx";"vy"])

%plot(t,[x;mu])
%legend(["x";"y";"theta";"vx";"vy";"x";"y";"theta";"vx";"vy"])

plot(t,[mu(1:2,:);u(1:2,:)])
legend(["x";"y";"ax";"ay"])
%plot(z(1,:),z(2,:))

err = norm(x-mu)
