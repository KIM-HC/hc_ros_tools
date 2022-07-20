clc
clear all


hz = 300;
dt = 1/hz;
w = 42.0;
a = dt / (1/w + dt);
% a = 0.999;
fprintf('a = %f\n',a);

t = 0:dt:100;
length = length(t);
real = sin(0.5*pi*t) + sin(0.02*pi*t);

lpf = zeros(1, length);

for k=2:length-1
    lpf(k) = a*lpf(k-1) + (1-a)*real(k);
end


plot(t,real, 'b')

hold on
grid on

plot(t, lpf, 'r')
legend('RAW','LPF')



hold off


