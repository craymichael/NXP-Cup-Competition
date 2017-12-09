close all;

MAX = 65;
MIN = 40;
angle = 0:180;
straightDur = 1:27;
duty = MAX ./ (abs(90 - angle) / 15);
straight = (straightDur/60)'*duty(80:100) + MAX/2;
straight = max(min(straight, MAX), MIN);
duty = max(min(duty, MAX), MIN);
subplot(2,1,1)
plot(angle,duty)
subplot(2,1,2)
plot(80:100, straight);

pwm = 5:0.01:10;
%motorduty = MAX-(MAX-MIN).*abs(7.5-pwm).^2/(10-7.5);
motorduty = MAX ./ (abs(7.5 - pwm)*5);
motorduty = max(min(motorduty, MAX), MIN);
figure
plot(pwm,motorduty);

figure
pwm = 5:0.01:10;
KI = 1:0.5:20;
dooty = min(max(100 ./ (abs(7.5-pwm') * KI), 50), 100);
plot(pwm,dooty)
xlabel('Servo PWM')
ylabel('DC PWM')
title('Turn-Dependent Speed Control')
legend(strtrim(cellstr(num2str(KI')))')

figure
dcpwm = MIN:MAX;
diff = MIN ./ max(abs(pwm-7.5)*2,1);
plot(pwm, diff)
hold on;
plot(pwm, MIN);

figure
diff = -MAX+MIN:MAX-MIN;
speed = MIN + diff.^2/531.2;
plot(diff, speed)

figure
indies = 0:0.5:127;
SC = 2.5 / (63.5.^2);
KP = 1:0.5:20;
error = (63.5 - indies);
dooty = min(max((-1* sign(error) .* SC .* error.^2)' * KP + 7.5, 5), 10);
plot(indies,dooty)
xlabel('Midpoint Index')
ylabel('Servo PWM')
title('Turn Control Theory')
legend(strtrim(cellstr(num2str(KP')))')