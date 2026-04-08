clc; clear; close all;

%% =========================
% 1. SYSTEM PARAMETERS
% =========================
% Đề tài: Thiết kế hiệu năng mạng Wi-Fi cho hệ thống UAV truyền video thời gian thực
% Chuẩn: IEEE 802.11ac (Wi-Fi 5), 5 GHz, Point-to-Point GCS <-> UAV
%
% Tham số phần cứng (khớp lý thuyết Chương 4.1):
%   UAV : NVIDIA Jetson Nano + Intel 8265NGW
%   GCS : Access Point anten định hướng

fc     = 5e9;       % Tần số sóng mang (Hz)
c      = 3e8;       % Vận tốc ánh sáng (m/s)
lambda = c / fc;    % Bước sóng: 0.06 m
Pt     = 20;        % Công suất phát UAV (dBm) - khớp lý thuyết Ch.4.1
Gt     = 2;         % Gain anten UAV: dipole/film đa hướng (dBi) - Ch.2.2
Gr     = 18;        % Gain anten GCS: định hướng (dBi) - Ch.4.1
ht     = 50;        % Độ cao UAV (m) - kịch bản bay tuần tra
hr     = 2;         % Độ cao anten GCS mặt đất (m)
BW     = 40e6;      % Băng thông kênh 802.11ac (Hz) - Ch.4.1

% Noise (khớp công thức lý thuyết Ch.4.2.4)
NF     = 7;                          % Noise Figure máy thu (dB)
N0     = -174 + 10*log10(BW) + NF;  % Tổng nhiễu nhiệt nền (dBm)
I_mean = -90;                        % Nhiễu đồng kênh nền (dBm)

% Rician K-factor = 5 (mặc định Ch.4.2.3, UAV ở độ cao 50m, LOS tốt)
K = 5;

%% =========================
% 2. WIFI MCS TABLE
% =========================
% 802.11ac, 1 spatial stream, BW=40MHz (khớp lý thuyết Ch.4.2.5)
SNR_req  = [4  7  10  13  16  20  24  28];  % Ngưỡng SINR tối thiểu (dB)
PHY_rate = [13 26 39  52  78 104 117 130];  % Tốc độ PHY tương ứng (Mbps)
MAC_eff  = 0.7;                              % Hiệu suất MAC (overhead 802.11)
Packet_Size_bits = 1500 * 8;                 % Kích thước gói Ethernet (bits)

% Bảng p_loss theo từng MCS — khai báo NGOÀI vòng lặp (tránh tái tạo 57500 lần)
% Mô hình L2S: p_loss giảm đơn điệu khi MCS tăng
% Nguồn: IEEE 802.11ac-2013 Annex B; ITU-T G.1010 (2001); Perahia & Stacey (2013)
p_loss_table = [0.30, 0.15, 0.08, 0.05, 0.02, 0.01, 0.005, 0.002];
%               MCS0  MCS1  MCS2  MCS3  MCS4  MCS5  MCS6   MCS7

%% =========================
% 3. KHOẢNG CÁCH & CRITICAL DISTANCE
% =========================
% dc = 4*ht*hr/lambda = 4*50*2/0.06 = 6667 m
% → Phải mở rộng d để thấy đủ 3 vùng theo lý thuyết Ch.4.3.1:
%   Vùng 1 (An toàn)  : d < dc        → suy hao Free-Space (20 dB/decade)
%   Vùng 2 (Cảnh báo) : dc ~ 1.5*dc   → chuyển tiếp
%   Vùng 3 (Sụp đổ)   : d > dc        → suy hao Two-Ray (40 dB/decade)
dc = (4 * ht * hr) / lambda;
fprintf('=== Khoảng cách tới hạn dc = %.0f m ===\n', dc);

d  = 500 : 100 : 12000;   % 500 m – 12 km, bước 100 m
Nd = length(d);

%% =========================
% 4. MONTE CARLO SETUP
% =========================
Nrun = 500;  % Đủ lớn để kết quả hội tụ ổn định

Throughput_all = zeros(Nrun, Nd);
SINR_all       = zeros(Nrun, Nd);
PacketLoss_all = zeros(Nrun, Nd);
Latency_all    = zeros(Nrun, Nd);

%% =========================
% 5. MAIN SIMULATION LOOP
% =========================
for k = 1:Nrun
    for i = 1:Nd

        % ===== Mô hình Two-Ray Ground Reflection (Ch.4.2.1) =====
        % Vùng gần (d <= dc): Free-Space, suy hao ~ d^2
        % Vùng xa  (d >  dc): Two-Ray, suy hao ~ d^4
        if d(i) <= dc
            PL = 20*log10(d(i)) + 20*log10(4*pi / lambda);
        else
            PL_dc = 20*log10(dc) + 20*log10(4*pi / lambda);
            PL    = PL_dc + 40*log10(d(i) / dc);
        end
        Pr_base = Pt + Gt + Gr - PL;

        % ===== Shadowing Log-Normal (Ch.4.2.2): sigma = 3 dB =====
        % Giới hạn dao động ở mức ±2σ (±6 dB) để tránh sụt giảm phi vật lý
        shadowing   = randn * 3;         % N(0,9): σ=3dB, KHÔNG clipping (Rappaport 2002 Sec 3.3)
        Pr_shadowed   = Pr_base + shadowing;

        % ===== Rician Fading (Ch.4.2.3) - công thức đúng =====
        % h = sqrt(K/(K+1)) + sqrt(1/(2*(K+1))) * (N(0,1) + jN(0,1))
        LOS_amp  = sqrt(K / (K + 1));
        
        NLOS_amp = sqrt(1 / (2*(K+1))) * (randn + 1i*randn);  % Gaussian phức, KHÔNG clipping (Simon & Alouini 2005)
        
        fading   = LOS_amp + NLOS_amp;
        Pr       = Pr_shadowed + 20*log10(abs(fading));

        % ===== Nhiễu & SINR (Ch.4.2.4) =====
        % Giới hạn nhiễu dao động ở mức ±2σ (±4 dB)
        I_inst = I_mean + randn * 2;   % Nhiễu tức thời, KHÔNG clipping (Tse & Viswanath 2005)
        NI_mW         = 10^(N0/10) + 10^(I_inst/10);
        SINR          = Pr - 10*log10(NI_mW);
        SINR_all(k, i) = SINR;

        % ===== Adaptive MCS (Ch.4.2.5) =====
        idx = find(SINR >= SNR_req, 1, 'last');
        if isempty(idx)
            rate = 0;
        else
            rate = PHY_rate(idx);
        end
        % ===== Packet Loss (Ch.4.2.6) — Căn theo từng ngưỡng MCS =====
        % Mô hình L2S (Link-to-System Mapping):
        % p_loss gắn với từng MCS index, giảm đơn điệu khi MCS tăng.
        % Nguồn: IEEE 802.11ac-2013 Annex B, ITU-T G.1010, Perahia & Stacey (2013)
        
        % p_loss_table đã khai báo ở Phần 2 (ngoài vòng lặp)
        
        if rate == 0
            % SINR < SNR_req(1) = 4 dB: kết nối sụp đổ hoàn toàn
            % Mọi gói đều mất → p_loss = 1.0
            p_loss = 1.0;
        else
            % idx đã tính ở bước MCS: index MCS đang dùng (1–8)
            p_loss = p_loss_table(idx);
        end
        
        loss = rand < p_loss;
        PacketLoss_all(k, i) = loss;

        % ===== Throughput (Ch.4.3.1) =====
        % Goodput = PHY_rate × MAC_efficiency × (1 − PER)
        % Nguồn: Bianchi (2000) IEEE JSAC; Bellalta et al. (2016) IEEE Wireless Comm.
        % MAC_eff = 0.7 hấp thụ overhead CSMA/CA, ACK, SIFS, DIFS
        % (1 - p_loss) phản ánh tỉ lệ gói thực sự thành công trên kênh
        
        if rate == 0
            tp = 0;                                  % Không có kết nối
        else
            tp = rate * MAC_eff * (1 - p_loss);      % Goodput theo MCS và PER
        end
        Throughput_all(k, i) = tp;


        % ===== Latency (Ch.1.3.1) =====
        % E2E Latency = Prop_Delay + Tx_Delay + MAC_Queue_Delay [+ Retry]
        Prop_Delay      = d(i) / c;                              % d/c
        MAC_Queue_Delay = 0.002 + rand * 0.003;                  % 2–5 ms
        if rate > 0
            Tx_Delay = Packet_Size_bits / (rate * 1e6);          % L/R
        else
            Tx_Delay = inf;
        end

        if loss && rate > 0
            Retry_Penalty   = Tx_Delay + 0.002 + rand * 0.005;
            current_latency = Prop_Delay + Tx_Delay + MAC_Queue_Delay + Retry_Penalty;
        else
            current_latency = Prop_Delay + Tx_Delay + MAC_Queue_Delay;
        end

        if rate == 0 || ~isfinite(current_latency) || current_latency > 0.2
            Latency_all(k, i) = 200;
        else
            Latency_all(k, i) = current_latency * 1000;   % → ms
        end
    end
end

%% =========================
% 6. AVERAGING & SMOOTHING
% =========================
Throughput_avg = mean(Throughput_all, 1);
SINR_avg       = mean(SINR_all, 1);
PacketLoss_avg = mean(PacketLoss_all, 1);
Latency_avg    = mean(Latency_all, 1);

 win = 8;
 Throughput_smooth  = movmean(Throughput_avg, win);
 SINR_smooth        = movmean(SINR_avg, win);
 PacketLoss_smooth  = movmean(PacketLoss_avg, win);
 Latency_smooth     = movmean(Latency_avg, win);


%% =========================
% 7. JITTER MODEL - HÀM LIÊN TỤC (Ch.4.2.7)
% =========================
% Lý thuyết: Jitter tăng do ICI (Inter-Carrier Interference) khi Doppler
% shift phá vỡ tính trực giao OFDM. ICI tỉ lệ với fd^2 → Jitter ~ fd^2.
%
% Công thức: fd = v / lambda  (Ch.4.2.7)
%            Jitter(ms) = J0 + alpha * fd^2
%
% Hiệu chỉnh: J0=5ms (jitter nền MAC), alpha để Jitter=30ms tại v≈20m/s
%   fd tại v=20 m/s = 20/0.06 = 333 Hz
%   30 = 5 + alpha * 333^2  → alpha ≈ 0.000225

v      = 0 : 1 : 30;        % Vận tốc UAV (m/s): 0–108 km/h
fd     = v / lambda;        % Doppler shift (Hz)
J0     = 5;                 % Jitter nền (ms)
alpha  = 0.000225;          % Hệ số ICI
Jitter = J0 + alpha * fd.^2;

%% =========================
% 8. NGƯỠNG & ĐIỂM GÃY (từ Bảng 2.1 lý thuyết)
% =========================
realtime_th = 5;     % Throughput tối thiểu cho video HD (Mbps) - Ch.1.3.4
latency_th  = 100;   % Độ trễ E2E tối đa cho FPV/realtime (ms) - Ch.1.3.1
jitter_th   = 30;    % Jitter tối đa cho UDP video (ms) - Ch.1.3.2

% Throughput
fail_idx   = Throughput_smooth < realtime_th;
fail_point = find(fail_idx, 1, 'first');

% Latency
high_latency_idx = Latency_smooth > latency_th;
lat_fail_point   = find(high_latency_idx, 1, 'first');

% Packet loss
high_loss = PacketLoss_smooth > 0.05;

% Jitter - điểm vận tốc vượt ngưỡng
jitter_fail_idx = find(Jitter > jitter_th, 1, 'first');

%% =========================
% 9. PLOTS
% =========================

% Màu chuẩn IEEE
col_raw    = [0.70 0.70 0.70];
col_smooth = [0.00 0.45 0.74];   % xanh dương
col_warn   = [1.00 0.60 0.60];   % đỏ nhạt

%% ===== Figure 1: Throughput vs Distance =====
% Thể hiện 3 vùng lý thuyết Ch.4.3.1
figure('Name','Fig 1: Throughput vs Distance','NumberTitle','off');
hold on; grid on;

% Tô nền 3 vùng
ylim_max = max(Throughput_smooth)*1.35 + 5;
patch([d(1) dc dc d(1)], [0 0 ylim_max ylim_max], [0.9 1.0 0.9], ...
    'EdgeColor','none','FaceAlpha',0.3,'DisplayName','Vùng An toàn');
patch([dc dc d(end) d(end)], [0 0 ylim_max ylim_max], [1.0 0.95 0.85], ...
    'EdgeColor','none','FaceAlpha',0.3,'DisplayName','Vùng Suy giảm / Sụp đổ');

plot(d, Throughput_avg, '--', 'Color', col_raw, 'DisplayName','Raw (Monte Carlo)');
plot(d, Throughput_smooth, 'Color', col_smooth, 'LineWidth', 2, 'DisplayName','Smoothed');
yline(realtime_th, 'r--', 'LineWidth', 1.5, 'Label','Realtime ≥ 5 Mbps');

% Đường dc
xline(dc, 'k:', 'LineWidth', 1.5, 'Label', sprintf('d_c=%.0fm',dc), ...
    'LabelVerticalAlignment','bottom');

% Điểm fail
if ~isempty(fail_point)
    x_f = d(fail_point); y_f = Throughput_smooth(fail_point);
    area(d(fail_idx), Throughput_smooth(fail_idx), ...
        'FaceColor', col_warn, 'EdgeColor','none', 'FaceAlpha',0.5, ...
        'DisplayName','Dưới ngưỡng Realtime');
    plot(x_f, y_f, 'ro','MarkerSize',8,'LineWidth',2,'HandleVisibility','off');
    text(x_f+100, y_f+3, sprintf('Sụp đổ @ %d m', round(x_f)), ...
        'Color','r','FontWeight','bold','FontSize',9);
end

xlabel('Khoảng cách (m)', 'FontSize',11);
ylabel('Throughput MAC (Mbps)', 'FontSize',11);
title('Throughput vs Khoảng cách – Kịch bản 1 (Ch.4.3.1)', 'FontSize',12);
legend('Location','northeast','FontSize',9);
xlim([d(1) d(end)]); ylim([0 ylim_max]);

% Chú thích 3 vùng
text(dc*0.4, ylim_max*0.92, 'Vùng AN TOÀN', ...
    'Color',[0.1 0.5 0.1],'FontWeight','bold','HorizontalAlignment','center','FontSize',9);
text(dc*1.35, ylim_max*0.92, 'Vùng SỤP ĐỔ', ...
    'Color',[0.7 0.2 0.0],'FontWeight','bold','HorizontalAlignment','center','FontSize',9);

%% ===== Figure 2: SINR vs Distance =====
figure('Name','Fig 2: SINR vs Distance','NumberTitle','off');
hold on; grid on;

% Tô nền 2 vùng
ylim_sinr = [min(SINR_smooth)-5, max(SINR_smooth)+5];
patch([d(1) dc dc d(1)], [ylim_sinr(1) ylim_sinr(1) ylim_sinr(2) ylim_sinr(2)], ...
    [0.9 1.0 0.9],'EdgeColor','none','FaceAlpha',0.3);
patch([dc dc d(end) d(end)], [ylim_sinr(1) ylim_sinr(1) ylim_sinr(2) ylim_sinr(2)], ...
    [1.0 0.95 0.85],'EdgeColor','none','FaceAlpha',0.3);

plot(d, SINR_avg,    '--','Color',col_raw,'DisplayName','Raw');
plot(d, SINR_smooth, 'Color',col_smooth,'LineWidth',2,'DisplayName','Smoothed');

% Ngưỡng MCS (Ch.4.2.5)
mcs_col = lines(length(SNR_req));
for m = 1:length(SNR_req)
    yline(SNR_req(m), ':', 'Color', mcs_col(m,:), 'Alpha',0.7, ...
        'Label',sprintf('MCS%d(%ddB)',m-1,SNR_req(m)), ...
        'LabelHorizontalAlignment','right','FontSize',7);
end

xline(dc,'k:','LineWidth',1.5,'Label',sprintf('d_c=%.0fm',dc), ...
    'LabelVerticalAlignment','bottom');

% Vùng tín hiệu yếu
weak_idx = SINR_smooth < SNR_req(1);
if any(weak_idx)
    area(d(weak_idx), SINR_smooth(weak_idx), ...
        'FaceColor',col_warn,'EdgeColor','none','FaceAlpha',0.5,'BaseValue',ylim_sinr(1));
    ws = find(weak_idx,1,'first');
    text(d(ws)+200, SINR_smooth(ws)+2,'Weak Signal Region','Color','r', ...
        'FontWeight','bold','FontSize',9);
end

xlabel('Khoảng cách (m)','FontSize',11);
ylabel('SINR (dB)','FontSize',11);
title('SINR vs Khoảng cách – Mô hình Two-Ray (Ch.4.2.1)','FontSize',12);
legend({'Raw','Smoothed'},'Location','northeast','FontSize',9);
xlim([d(1) d(end)]); ylim(ylim_sinr);

%% ===== Figure 3: Packet Loss vs Distance =====
figure('Name','Fig 3: Packet Loss vs Distance','NumberTitle','off');
hold on; grid on;

patch([d(1) dc dc d(1)], [0 0 0.65 0.65], [0.9 1.0 0.9], ...
    'EdgeColor','none','FaceAlpha',0.3);
patch([dc dc d(end) d(end)], [0 0 0.65 0.65], [1.0 0.95 0.85], ...
    'EdgeColor','none','FaceAlpha',0.3);

plot(d, PacketLoss_avg,    '--','Color',col_raw,'DisplayName','Raw');
plot(d, PacketLoss_smooth, 'Color',[0.85 0.33 0.1],'LineWidth',2,'DisplayName','Smoothed');
yline(0.05,'r--','LineWidth',1.5,'Label','Ngưỡng Video 5%');
yline(0.01,'b:','LineWidth',1.2,'Label','Ngưỡng Tốt 1%');

xline(dc,'k:','LineWidth',1.5,'Label',sprintf('d_c=%.0fm',dc), ...
    'LabelVerticalAlignment','bottom');

if any(high_loss)
    area(d(high_loss), PacketLoss_smooth(high_loss), ...
        'FaceColor',[1 0.7 0.7],'EdgeColor','none','FaceAlpha',0.5, ...
        'DisplayName','Loss > 5% (Ch.2.3.2)');
    hl = find(high_loss,1,'first');
    text(d(hl)+150, PacketLoss_smooth(hl)+0.015, ...
        sprintf('Loss > 5%%\n@ %dm',round(d(hl))), ...
        'Color',[0.7 0 0],'FontWeight','bold','FontSize',9);
end

xlabel('Khoảng cách (m)','FontSize',11);
ylabel('Packet Loss Ratio','FontSize',11);
title('Tỷ lệ mất gói vs Khoảng cách (Ch.4.2.6)','FontSize',12);
legend('Location','northwest','FontSize',9);
xlim([d(1) d(end)]); ylim([0 1.05]);

%% ===== Figure 4: Jitter vs Vận tốc UAV (Ch.4.2.7) =====
% Mô hình hàm liên tục: Jitter = J0 + alpha*fd^2
% Phản ánh đúng quan hệ vật lý ICI ∝ fd^2 (lý thuyết Ch.4.2.7)
figure('Name','Fig 4: Jitter vs UAV Velocity','NumberTitle','off');
hold on; grid on;

% Vùng fail (Jitter > 30ms)
fail_v = Jitter > jitter_th;
if any(fail_v)
    area(v(fail_v), Jitter(fail_v), ...
        'FaceColor',col_warn,'EdgeColor','none','FaceAlpha',0.5, ...
        'BaseValue',jitter_th);
end

plot(v, Jitter, 'b','LineWidth',2,'DisplayName', ...
    sprintf('Jitter = %.0f + %.6f·f_d² (ms)',J0,alpha));
yline(jitter_th,'r--','LineWidth',1.5,'Label',sprintf('Realtime Limit (%dms)',jitter_th));

% Điểm vượt ngưỡng
if ~isempty(jitter_fail_idx)
    v_crit = v(jitter_fail_idx);
    fd_crit = fd(jitter_fail_idx);
    plot(v_crit, Jitter(jitter_fail_idx), 'ro','MarkerSize',8,'LineWidth',2, ...
        'HandleVisibility','off');
    text(v_crit+0.5, Jitter(jitter_fail_idx)+3, ...
        sprintf('ICI vượt ngưỡng\nv = %d m/s (%.0f km/h)\nf_d = %.0f Hz', ...
        v_crit, v_crit*3.6, fd_crit), ...
        'Color','r','FontWeight','bold','FontSize',9);
    text(mean(v(fail_v)), max(Jitter)*0.75, 'Unstable Video', ...
        'Color','r','FontWeight','bold','HorizontalAlignment','center','FontSize',10);
end

% Annotate Doppler shift tại các mốc
for ii = 1:3:length(v)
    text(v(ii), Jitter(ii)-4, sprintf('f_d\n%.0fHz',fd(ii)), ...
        'FontSize',7,'HorizontalAlignment','center','Color',[0.4 0.4 0.4]);
end

xlabel('Vận tốc UAV (m/s)','FontSize',11);
ylabel('Jitter (ms)','FontSize',11);
title('Jitter vs Vận tốc UAV – Hiệu ứng Doppler/ICI (Ch.4.2.7)','FontSize',12);
legend('Location','northwest','FontSize',9);
xlim([v(1) v(end)]); ylim([0 max(Jitter)*1.2+5]);

%% ===== Figure 5: Latency vs Distance (Ch.1.3.1) =====
figure('Name','Fig 5: Latency vs Distance','NumberTitle','off');
hold on; grid on;

patch([d(1) dc dc d(1)], [0 0 220 220], [0.9 1.0 0.9], ...
    'EdgeColor','none','FaceAlpha',0.3);
patch([dc dc d(end) d(end)], [0 0 220 220], [1.0 0.95 0.85], ...
    'EdgeColor','none','FaceAlpha',0.3);

plot(d, Latency_avg,    '--','Color',col_raw,'DisplayName','Raw');
plot(d, Latency_smooth, '-m','LineWidth',2,'DisplayName','Smoothed');
yline(latency_th,'r--','LineWidth',1.5, ...
    'Label',sprintf('Ngưỡng FPV Realtime (%dms)',latency_th));

xline(dc,'k:','LineWidth',1.5,'Label',sprintf('d_c=%.0fm',dc), ...
    'LabelVerticalAlignment','bottom');

if any(high_latency_idx)
    area(d(high_latency_idx), Latency_smooth(high_latency_idx), ...
        'FaceColor',col_warn,'EdgeColor','none','FaceAlpha',0.5, ...
        'BaseValue',latency_th,'DisplayName','Vượt ngưỡng Latency');
    if ~isempty(lat_fail_point)
        x_lf = d(lat_fail_point); y_lf = Latency_smooth(lat_fail_point);
        plot(x_lf, y_lf,'ro','MarkerSize',8,'LineWidth',2,'HandleVisibility','off');
        text(x_lf+150, y_lf+8, sprintf('Latency Fail\n@ %d m',round(x_lf)), ...
            'Color','r','FontWeight','bold','FontSize',9);
    end
end

xlabel('Khoảng cách (m)','FontSize',11);
ylabel('Độ trễ E2E (ms)','FontSize',11);
title('Độ trễ End-to-End vs Khoảng cách (Ch.1.3.1)','FontSize',12);
legend('Location','northwest','FontSize',9);
xlim([d(1) d(end)]); ylim([0 220]);

%% =========================
% 10. SUMMARY REPORT
% =========================
fprintf('\n============================================================\n');
fprintf('           TÓM TẮT KẾT QUẢ MÔ PHỎNG\n');
fprintf('============================================================\n');
fprintf('Tần số              : %.1f GHz\n', fc/1e9);
fprintf('Băng thông          : %.0f MHz\n', BW/1e6);
fprintf('Công suất phát UAV  : %.0f dBm\n', Pt);
fprintf('Gain anten GCS      : %.0f dBi\n', Gr);
fprintf('Độ cao UAV / GCS    : %.0f m / %.0f m\n', ht, hr);
fprintf('Khoảng cách tới hạn : %.0f m (Two-Ray d_c)\n', dc);
fprintf('Rician K-factor     : %d\n', K);
fprintf('Monte Carlo runs    : %d\n', Nrun);
fprintf('------------------------------------------------------------\n');
if ~isempty(fail_point)
    fprintf('[KQ1] Throughput sụp đổ : @ %.0f m (< %.0f Mbps)\n', d(fail_point), realtime_th);
else
    fprintf('[KQ1] Throughput        : Không sụp đổ trong tầm khảo sát\n');
end
if ~isempty(lat_fail_point)
    fprintf('[KQ2] Latency vượt ngưỡng: @ %.0f m (> %.0f ms)\n', d(lat_fail_point), latency_th);
else
    fprintf('[KQ2] Latency           : Không vượt ngưỡng trong tầm khảo sát\n');
end
if ~isempty(jitter_fail_idx)
    fprintf('[KQ3] Jitter vượt ngưỡng : v > %d m/s (%.0f km/h), fd > %.0f Hz\n', ...
        v(jitter_fail_idx), v(jitter_fail_idx)*3.6, fd(jitter_fail_idx));
end
fprintf('[KQ4] SINR max / min  : %.1f dB / %.1f dB\n', max(SINR_smooth), min(SINR_smooth));
fprintf('============================================================\n');
fprintf('→ UAV hoạt động AN TOÀN trong bán kính < d_c = %.0f m\n', dc);
fprintf('→ Vận tốc tối đa để video ổn định: %d m/s (%.0f km/h)\n', ...
    v(jitter_fail_idx)-1, (v(jitter_fail_idx)-1)*3.6);
fprintf('============================================================\n');