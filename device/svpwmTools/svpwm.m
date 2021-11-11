function yi = svpwm(xResolution, yResolution)
% 生成 SVPWM 加速表
% xResolution 时间分辨率, 例如 xResolution = 256 则表示一个正弦波周期
% 有 6 * 256 个点。
% yResolution 幅值分辨率, 例如 yResolution = 255 则表示整项伯最大值是
% 255。
% 返回加速表

ua = zeros(1, 6 * xResolution);
ub = zeros(1, 6 * xResolution);
uc = zeros(1, 6 * xResolution);

j = 1;
for sector = 0 : 1 : 6 - 1
    for i = 0 : 1 : xResolution - 1
        a = sqrt(3) * cos((pi / 3) * sector + i * (pi / 3 / xResolution));
        b = sin((pi / 3) * sector + i * (pi / 3 / xResolution));
        X = b;
        Y = (a + b) / 2;
        Z = (b - a) / 2;
        if sector == 0
            ua(1, j) = 0.5 + (X - Z) / 2;
            ub(1, j) = ua(1, j) + Z;
            uc(1, j) = ub(1, j) - X;
        end
        if sector == 1
            ua(1, j) = 0.5 + (Y - Z) / 2;
            ub(1, j) = ua(1, j) + Z;
            uc(1, j) = ua(1, j) - Y;
        end
        if sector == 2
            ua(1, j) = 0.5 + (Y - X) / 2;
            uc(1, j) = ua(1, j) - Y;
            ub(1, j) = uc(1, j) + X;
        end
        if sector == 3
            ua(1, j) = 0.5 + (X - Z) / 2;
            ub(1, j) = ua(1, j) + Z;
            uc(1, j) = ub(1, j) - X;
        end
        if sector == 4
            ua(1, j) = 0.5 + (Y - Z) / 2;
            ub(1, j) = ua(1, j) + Z;
            uc(1, j) = ua(1, j) - Y;
        end
        if sector == 5
            ua(1, j) = 0.5 + (Y - X) / 2;
            uc(1, j) = ua(1, j) - Y;
            ub(1, j) = uc(1, j) + X;
        end
        ua(1, j) = round(ua(1, j) * yResolution);
        ub(1, j) = round(ub(1, j) * yResolution);
        uc(1, j) = round(uc(1, j) * yResolution);
        j = j + 1;
    end
end

plot(ua,'Color',[1 1 0]);
hold on;
plot(ub,'Color',[0 1 0]);
plot(uc,'Color',[1 0 0]);
hold off;

yi = ua(1, xResolution * 0.5 + 1 : xResolution * 1.5);

end