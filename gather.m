%% Leader-follower Multi-Agent System, V shape Formation Control + Obstacle Advoidance using APF and RPF

% Since RPF is nonlinear function, and the graph is unbalanced, the control
% law can't be combined RPF with APF (linear) in an array
clear all;close all;clc;

global eta d dt iteration numNodes z0 kd k_x k_y

numNodes = 5; %number of agents

eta =6; %gain of RPF

kd = 10;% gain of APF

d = 10; % The distance whcih robots will be in dancger of collision (activated zone)

dt=0.01;%sampling period

iteration =600;%numbers of iterations 

z0 = [5 18 8 7 10 2 13 15 16 19]'%initial condition
%desired relative position

c12 = [-1,-1];
c13 = [1,-1];
c23 = [2,0];
c24 = [-1,-1];
c32 = [-2,0];
c35 = [1,-1];
c45 = [4,0];
c54 = [-4,0];
%desired vectors of relative positions (c_mat = [sum of the desired vectors of positions for x1, y1, x2, y2, x3, y3, x4, y4])
c_mat = [0,...
         0,...
        c12(1,1) + c32(1,1),... 
        c12(1,2) + c32(1,2),... 
        c13(1,1) + c23(1,1),... 
        c13(1,2) + c23(1,2),... 
        c24(1,1) + c54(1,1),... 
        c24(1,2) + c54(1,2),... 
        c35(1,1) + c45(1,1),... 
        c35(1,2) + c45(1,2)]';
% c_mat = [c21(1,1)+c31(1,1), c21(1,2)+c31(1,2), c12(1,1), c12(1,2), c23(1,1), c23(1,2), c34(1,1), c34(1,2)]';

%initialize the array for the state of agents
%z = [z1x(0) z1x(k) ...
%       z1y(0) z1y(k) ......
%       .
%       .
%       z8y(0) z8y(k)]
%size(z) = [numNodes*2, k]
z = zeros(numNodes*2, iteration+1);

%Initialize the array for the distance between each 'adjacent' agents
% dis = [  0   dis12 dis13 dis14;
%        dis21   0   dis23 dis24;
%        dis31 dis32   0   dis34;
%        dis41 dis42 dis43   0  ...];
dis = zeros(numNodes,numNodes,length(z));

%Initialize the array for the RPF's derivative
% d_v = [  0   dv_12 dv_13 dv_14;
%        dv_21   0   dv_23 dv_24;
%        dv_31 dv_32   0   dv_34;
%        dv_41 dv_42 dv_43   0  ...];
d_v =zeros(numNodes,numNodes,length(z));
z(:,1)=z0; 
for k =1:iteration+1
    
    dis(1,2,k) = norm(z(1:2,k) - z(3:4,k)); %distance between node 1 and node 2
    dis(1,3,k) = norm(z(1:2,k) - z(5:6,k)); %distance between node 1 and node 3
    dis(1,4,k) = norm(z(1:2,k) - z(7:8,k)); %distance between node 1 and node 4
    dis(1,5,k) = norm(z(1:2,k) - z(9:10,k));
    dis(2,1,k) = dis(1,2,k);                    %distance between node 2 and node 1 (same as dis21)
    dis(2,3,k) = norm(z(3:4,k) - z(5:6,k)); %distance between node 2 and node 3
    dis(2,4,k) = norm(z(3:4,k) - z(7:8,k)); %distance between node 2 and node 4
    dis(2,5,k) = norm(z(3:4,k) - z(9:10,k));
    dis(3,1,k) = dis(1,3,k);                    %distance between node 3 and node 1
    dis(3,2,k) = dis(2,3,k);                    %distance between node 3 and node 2
    dis(3,4,k) = norm(z(5:6,k) - z(7:8,k)); %distance between node 3 and node 4
    dis(3,5,k) = norm(z(5:6,k) - z(9:10,k));
    dis(4,1,k) = dis(1,4,k);                    %distance between node 4 and node 1
    dis(4,2,k) = dis(2,4,k);                    %distance between node 4 and node 2
    dis(4,3,k) = dis(3,4,k);                    %distance between node 4 and node 3
    dis(4,5,k) = norm(z(7:8,k) - z(9:10,k));
    dis(5,1,k) = dis(1,5,k);
    dis(5,2,k) = dis(2,5,k);
    dis(5,3,k) = dis(3,5,k);
    dis(5,4,k) = dis(4,5,k);
    
    d_v(1,2,k) = -4*eta*(d^2 - dis(1,2,k)^2) / (d^2*dis(1,2,k)^5); %potential derivative of node 1 to 2
    d_v(2,1,k) = -d_v(2,1,k);                                      %potential derivative of node 2 to 1
    d_v(1,3,k) = -4*eta*(d^2 - dis(1,3,k)^2) / (d^2*dis(1,3,k)^5); %potential derivative of node 1 to 3
    d_v(3,1,k) = -d_v(1,3,k);
    d_v(1,4,k) = -4*eta*(d^2 - dis(1,4,k)^2) / (d^2*dis(1,4,k)^5);
    d_v(4,1,k) = -d_v(1,4,k);
    d_v(1,5,k) = -4*eta*(d^2 - dis(1,5,k)^2) / (d^2*dis(1,5,k)^5);
    d_v(5,1,k) = -d_v(1,5,k);
    d_v(2,3,k) = -4*eta*(d^2 - dis(2,3,k)^2) / (d^2*dis(2,3,k)^5);
    d_v(3,2,k) = -d_v(2,3,k);
    d_v(2,4,k) = -4*eta*(d^2 - dis(2,4,k)^2) / (d^2*dis(2,4,k)^5);
    d_v(4,2,k) = -d_v(2,4,k);
    d_v(2,5,k) = -4*eta*(d^2 - dis(2,5,k)^2) / (d^2*dis(2,5,k)^5);
    d_v(5,2,k) = -d_v(2,5,k);
    d_v(3,4,k) = -4*eta*(d^2 - dis(3,4,k)^2) / (d^2*dis(3,4,k)^5);
    d_v(4,3,k) = -d_v(3,4,k);
    d_v(3,5,k) = -4*eta*(d^2 - dis(3,5,k)^2) / (d^2*dis(3,5,k)^5);
    d_v(5,3,k) = -d_v(3,5,k);
    d_v(4,5,k) = -4*eta*(d^2 - dis(4,5,k)^2) / (d^2*dis(4,5,k)^5);
    d_v(5,4,k) = -d_v(4,5,k);
    % if-else condition, make sure that RPF starts to activate when obstacles enter the danger zone
    % if (current distance)^2 - d^2 <=0
    %    de = 1 
    % else
    %    de = 0
    de12 = (dis(1,2,k)^2 - d^2 <= 0);
    de13 = (dis(1,3,k)^2 - d^2 <= 0);
    de14 = (dis(1,4,k)^2 - d^2 <= 0);
    de15 = (dis(1,5,k)^2 - d^2 <= 0);
    de23 = (dis(2,3,k)^2 - d^2 <= 0);
    de24 = (dis(2,4,k)^2 - d^2 <= 0);
    de25 = (dis(2,5,k)^2 - d^2 <= 0);
    de34 = (dis(3,4,k)^2 - d^2 <= 0);
    de35 = (dis(3,5,k)^2 - d^2 <= 0);
    de45 = (dis(4,5,k)^2 - d^2 <= 0);
    
    % second stage: move together in certain pattern
    if ((z(3,k) - z(1,k)) + (z(3,k) - z(5,k)) - c_mat(3,1) < 0.5 ...
            && (z(4,k) - z(2,k) + (z(4,k) - z(6,k)) - c_mat(4,1) <0.5)...
            && (z(5,k) - z(1,k) + (z(5,k) - z(3,k)) - c_mat(5,1) < 0.5)...
            && (z(6,k) - z(2,k) + (z(6,k) - z(4,k)) - c_mat(6,1) < 0.5)...
            && (z(7,k) - z(3,k) + (z(7,k) - z(9,k)) - c_mat(7,1) < 0.5)...
            && (z(8,k) - z(4,k) + (z(8,k) - z(10,k)) - c_mat(8,1) < 0.5)...
            && (z(9,k) - z(5,k) + (z(9,k) - z(7,k)) - c_mat(9,1) < 0.5)...
            && (z(10,k) - z(6,k) + (z(10,k) - z(8,k)) - c_mat(10,1) < 0.5))
        % leader bot moving condition
        if (k <= 100)
            k_x = 1;
            k_y = 1.005;
        elseif (k > 100 && k <= 150)
            k_x = 1.005;
            k_y = 1.005;
        elseif (k > 150 && k <= 200)
            k_x = 1;
            k_y = 1.005;
        elseif (k > 200 && k <= 250)
            k_x = 0.995;
            k_y = 1.005;
        elseif (k > 250 && k <= 300)
            k_x = 1.005;
            k_y = 1.005;
        elseif (k > 300)
            k_x = 1;
            k_y = 1.005;           
        end
    else
        k_x = 1;
        k_y = 1.001;  
    end
  
    %nonlinear equation of the agent dynamic (z(k+1) = z(k) - APF's*dt - RPF's*dt))
    z(1,k+1) = k_x * z(1,k); 
    
    z(2,k+1) = k_y * z(2,k);
    
    z(3,k+1) = z(3,k) - de12*d_v(2,1,k)*dt - de23*d_v(2,3,k)*dt - de24*d_v(2,4,k)*dt - de25*d_v(2,5,k)*dt...
               - kd*( (z(3,k) - z(1,k)) + (z(3,k) - z(5,k)) - c_mat(3,1) )*dt; %z2_x
    
    z(4,k+1) = z(4,k) - de12*d_v(2,1,k)*dt - de23*d_v(2,3,k)*dt - de24*d_v(2,4,k)*dt - de25*d_v(2,5,k)*dt...
               - kd*( (z(4,k) - z(2,k)) + (z(4,k) - z(6,k)) - c_mat(4,1) )*dt; %z2_y
    
    z(5,k+1) = z(5,k) - de13*d_v(3,1,k)*dt - de23*d_v(3,2,k)*dt - de34*d_v(3,4,k)*dt - de35*d_v(3,5,k)*dt...
               - kd*( (z(5,k) - z(1,k)) + (z(5,k) - z(3,k)) - c_mat(5,1) )*dt; %z3_x
    
    z(6,k+1) = z(6,k) - de13*d_v(3,1,k)*dt - de23*d_v(3,2,k)*dt - de34*d_v(3,4,k)*dt - de35*d_v(3,5,k)*dt...
               - kd*( (z(6,k) - z(2,k)) + (z(6,k) - z(4,k)) - c_mat(6,1) )*dt; %z3_y
    
    z(7,k+1) = z(7,k) - de14*d_v(4,1,k)*dt - de24*d_v(4,2,k)*dt - de34*d_v(4,3,k)*dt - de45*d_v(4,5,k)*dt...
               -kd*( (z(7,k) - z(3,k)) + (z(7,k) - z(9,k)) - c_mat(7,1) )*dt; %z4_x
    
    z(8,k+1) = z(8,k) - de14*d_v(4,1,k)*dt - de24*d_v(4,2,k)*dt - de34*d_v(4,3,k)*dt - de45*d_v(4,5,k)*dt...
               -kd*( (z(8,k) - z(4,k)) + (z(8,k) - z(10,k)) - c_mat(8,1) )*dt; %z4_y
    
    z(9,k+1) = z(9,k) - de15*d_v(5,1,k)*dt - de25*d_v(5,2,k)*dt - de35*d_v(5,3,k)*dt - de45*d_v(5,4,k)*dt...
               -kd*( (z(9,k) - z(5,k)) + (z(9,k) - z(7,k)) - c_mat(9,1) )*dt; %z4_x
    
    z(10,k+1) = z(10,k) - de15*d_v(5,1,k)*dt - de25*d_v(5,2,k)*dt - de35*d_v(5,3,k)*dt - de45*d_v(5,4,k)*dt...
                -kd*( (z(10,k) - z(6,k)) + (z(10,k) - z(8,k)) - c_mat(10,1) )*dt; %z4_y

    
end

figure(2)
plot(z(1,1),z(2,1),'ro',z(3,1),z(4,1),'bo',z(5,1),z(6,1),'go',z(7,1),z(8,1),'ko',z(9,1),z(10,1),'yo')
hold on
for i=2:iteration
    pause(0.005);
    fprintf('%d\n', i);
    plot(z(1,i),z(2,i),'r:.')
    hold on
    plot(z(3,i),z(4,i),'b:.')
    hold on
    plot(z(5,i),z(6,i),'g:.')
    hold on
    plot(z(7,i),z(8,i),'k:.')
    hold on 
    plot(z(9,i),z(10,i),'y:.')
    drawnow
    hold on
    grid on
    xlabel('x')
    ylabel('y')
    title('Formation Control and Collision Advoidance using APF and RPF (Digraph) (o=start, x=end)')
    grid on
end
plot(z(1,iteration+1), z(2,iteration+1),'xr',z(3,iteration+1), z(4,iteration+1),'xb',...
     z(5,iteration+1), z(6,iteration+1),'xg',z(7,iteration+1), z(8,iteration+1),'xk',...
     z(9,iteration+1),z(10,iteration+1),'xy');
legend('Agent1','Agent2','Agent3','Agent4','Agent5')