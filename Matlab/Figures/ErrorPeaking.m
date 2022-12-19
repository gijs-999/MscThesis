N = 500;
dN = 50;
T = 2.5;
dt = T/N;
Lines = lines(8);
Lines(8,:) = [1,1,1];

t = (dt:dt:T+dN*dt+dt)';

v_ = 0.75;
v_ = v_ + 0.1*cos(t+0.5)
v_ = v_-0.25*[zeros(N/2,1);ones(N/2+dN+1,1)]


i_I{1} = N/2-dN;
i_I{2} = N/2;

v{1} = v_;
v{2} = circshift(v_,-dN)-0.05;
v{3} = v{1}-v{2};
figure(1); clf; hold on

for i = 1:3
  
plot(0,0, 'color',Lines(i,:))
end

for i = 1:3
  
plot(t(1:i_I{1}),v{i}(1:i_I{1}), 'color',Lines(i,:))
plot(t(i_I{1}+1:i_I{2}),v{i}(i_I{1}+1:i_I{2}), 'color',Lines(i,:))
plot(t(i_I{2}+1:end),v{i}(i_I{2}+1:end), 'color',Lines(i,:))
end

for i = 1:2
plot(t(i_I{i})*[1,1],[0,1],':k')
end

legend({'$v$','$v_{ref}$','$v_{ref}-v$'})
ylim([0,1])
xlim([0,T])


w = 700;
figure(1);
box on;
f{1}.Position =[10 10 w 440] ;

%% extend references
