%x = [o1,o_dot_1,o2,o_dot_2]
A = [0,1,0,0;0,0,0,0;0,0,0,1;0,0,0,0];
eig(A);
% all eigenvalues are 0 => system is unstable we need to introduce control
% say we can control o_1_dot_1
B = [0;1;0;0];
%lets check the controllability
r_c = rank(ctrb(A,B));
%the rank of the controllability matrix = 2 < 4 => the system is not
%controllable so we need better actuators
% say we can control o_2_dot_2
B= [0,0;1,0;0,0;0,1];
% lets check the controllability 
rank(ctrb(A,B));
%rank = 4 => controllable 
% now we can do state feedback to stabilize the system
% u = -k *x
%we need to determine the values of  k to match our desired eigenvaleus
%let's say we want as new eigenvalues -1,-2,-1,-2
P=[-1,-2,-1,-2];
K = place(A,B,P);
A_prime = A-B*K;
% now we have a new A and a stabilized system 
%now we need to create an observer to determine x cus actually we dont have
%x so we need to create an observer 
C = [1,0,0,0;0,0,1,0];
%let's check the observability 
rank(obsv(A,C));
% the rank is 4 so we do have an observable system so we can determine x^
P1 = [-3;-4;-3;-4];
L = place(A',C',P1).';
A_e = A-L*C;
O=zeros(size(A));
A_s = [A-B*K,B*K;O,A-L*C]
% now let's execute
x=[-5;1;1;1];x_hat=[0;0;0;0];t=0;tf=5;dt=0.01;
X=[];T=[];X_hat=[];
while(t<tf)
        X=[X,x];T=[T;t];X_hat=[X_hat,x_hat];
        x= x+dt.*(A-B*K)*x;
        x_hat=x_hat+dt.*((A-B*K)*x_hat +L*(C*x - C*x_hat)); 
        t=t+dt;
end
hold on ;

plot(T,X(1,:));
plot(T,X_hat(1,:));
%as we can see x _hat converges to x 





