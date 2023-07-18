        clf;
%% Acquire data
x_opt = output.result.solution.phase.state(:,1);
y_opt = output.result.solution.phase.state(:,2);
theta_opt = output.result.solution.phase.state(:,3);
u1_opt = output.result.solution.phase.control(:,1); %velocity
u2_opt = output.result.solution.phase.control(:,2); %theta_dot
t = output.result.solution.phase.time;


%%

figure(100)
subplot(1,2,1)
plot(x_opt,y_opt,'*','LineWidth',2)
 hold on
% comment appropriately based on NLP guess
plot([1; 2.75; 6.75; 9],[0.5; 7; 3; 9.5],'r--')
% plot([1; 1.25; 1.25; 1.5; 4.75; 4.125; 3.625; 3.625; 3.625; 4.75; 8.25; 8.5; 8.5; 9],[0.5; 3.5; 8.5; 8.5; 8.5; 6.25; 6.25; 5.75; 2.25; 1.5; 1.5; 1.5; 6.5; 9.5],'r--')
% plot([1; 1.5; 3.5; 4; 8; 9],[0.5; 8; 8; 3; 2; 9.5],'r--')
xlabel('x-position','FontSize', 16)
ylabel('y-position','FontSize', 16)
title('Basic Shape Min Time Solution','FontSize', 16)
 axis([0 10 0 10])
 legend('Optimal Path','Initial Guess','Location','NorthWest')
axis square

%% Plot first constraint
x0 = 2.75;
y0 = 3.5;
a = 1;
b = 1;
c = 1;
p = 2;
n = p/2;
th = linspace(0,3*pi,100000);
x = abs(cos(th)).^(1/n).*c*a.*sign(cos(th))+x0;
y = abs(sin(th)).^(1/n).*c*b.*sign(sin(th))+y0;
plot(x,y,'k')

%% Plot Second constraint
x0 = 5;
y0 = 5;
a = 1;
b = 1;
c = 1;
p = 2;
n = p/2;
th = linspace(0,3*pi,1000);
x = abs(cos(th)).^(1/n).*c*a.*sign(cos(th))+x0;
y = abs(sin(th)).^(1/n).*c*b.*sign(sin(th))+y0;
plot(x,y,'k')

%% Plot third constraint
x0 = 6.75;
y0 = 6.5;
a = 1;
b = 1;
c = 1;
p = 2;
n = p/2;
th = linspace(0,3*pi,1000);
x = abs(cos(th)).^(1/n).*c*a.*sign(cos(th))+x0;
y = abs(sin(th)).^(1/n).*c*b.*sign(sin(th))+y0;
plot(x,y,'k')

figure(100)
subplot(1,2,2)
plot(t,u2_opt,'LineWidth',2)
xlabel('Time (sec)')
ylabel('Heading Rate (deg)')
title('Min Time; Multi-Phase Triangular Control')
axis([0 25 -1 1])
axis square