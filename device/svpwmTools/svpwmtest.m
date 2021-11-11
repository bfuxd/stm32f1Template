function svpwmtest(wave)
% SVPWM 加速表测试
% wave 是由svpwm.m 的返回值

PHASE_S = length(wave);
PWM_MAX = max(wave);
ua = zeros(1, PHASE_S * 6);
ub = zeros(1, PHASE_S * 6);
uc = zeros(1, PHASE_S * 6);
t = fix(PHASE_S / 2);
for phase = 0 : 1 : PHASE_S * 6 - 1
    section = fix(phase / (PHASE_S * 2));
    phase_ = phase - section * (PHASE_S * 2) + 1;
    if(phase_ < t)
        a = wave(1, t - phase_);
        b = PWM_MAX - wave(1, phase_ + t);
        c = PWM_MAX - a;
    end
    if(phase_ < (PHASE_S + t)) && (phase_ >= t)
        a = wave(1, phase_ - t + 1);
        b = wave(1, PHASE_S + t - phase_);
        if(phase_ < PHASE_S)
            c = PWM_MAX - a;
        else
            c = PWM_MAX - b;
        end
    end
    if(phase_ >= (PHASE_S + t))
        a = PWM_MAX - wave(1, (PHASE_S * 2) + t - phase_);
        b = wave(1, phase_ - PHASE_S - t + 1);
        c = PWM_MAX - b;
    end
    if(section == 0)
        ua(1, phase + 1) = a;
        ub(1, phase + 1) = b;
        uc(1, phase + 1) = c;
    end
    if(section == 1)
        ua(1, phase + 1) = c;
        ub(1, phase + 1) = a;
        uc(1, phase + 1) = b;
    end
    if(section == 2)
        ua(1, phase + 1) = b;
        ub(1, phase + 1) = c;
        uc(1, phase + 1) = a;
    end
end
plot(ua,'Color',[1 1 0]);
hold on;
plot(ub,'Color',[0 1 0]);
plot(uc,'Color',[1 0 0]);
hold off;
end

