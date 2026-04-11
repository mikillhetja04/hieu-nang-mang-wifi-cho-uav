clc; clear; close all;
rng(42);   % Seed cố định — đảm bảo kết quả tái lập được khi bảo vệ đề tài
           % Nguồn: MathWorks, rng documentation

%% =========================
% 1. SYSTEM PARAMETERS
% =========================
% Đề tài: Thiết kế và đánh giá hiệu năng mạng Wi-Fi cho hệ thống UAV truyền video thời gian thực
% Chuẩn: IEEE 802.11ac (Wi-Fi 5), băng tần 5 GHz, kiến trúc Point-to-Point GCS <-> UAV
%
% Tham số phần cứng (khớp lý thuyết Chương 4.1):
%   UAV : NVIDIA Jetson Nano + Intel 8265NGW (module Wi-Fi M.2, 22×30mm)
%   GCS : Access Point tĩnh với anten định hướng độ lợi cao
%
% Hai kịch bản mô phỏng (Ch.4.3):
%   Kịch bản 1 — Biến độc lập: khoảng cách d (100–12000 m), vận tốc UAV = 0
%                Đánh giá: Throughput, SINR, Latency theo khoảng cách
%                Xuất: Figure 1, 2, 3
%   Kịch bản 2 — Biến độc lập: vận tốc v (0–30 m/s), khoảng cách cố định = 2000 m
%                Đánh giá: Jitter luồng UDP video do hiệu ứng Doppler/ICI
%                Xuất: Figure 4

fc     = 5e9;       % Tần số sóng mang (Hz) — băng 5 GHz, ít nhiễu, BW lớn (Ch.4.1)
c      = 3e8;       % Vận tốc ánh sáng (m/s)
lambda = c / fc;    % Bước sóng: lambda = 0.06 m tại 5 GHz
Pt     = 20;        % Công suất phát UAV (dBm) — giới hạn bởi pin Li-Po (Ch.4.1, Ch.2.2)
Gt     = 3;         % Độ lợi anten UAV: dipole/film đa hướng (dBi) — Ch.2.2
Gr     = 15;        % Độ lợi anten GCS: anten định hướng (dBi) — Ch.4.1
ht     = 50;        % Độ cao bay UAV (m) — kịch bản giám sát/tuần tra cố định
hr     = 2;         % Độ cao anten GCS tại mặt đất (m)
BW     = 40e6;      % Băng thông kênh 802.11ac (Hz) — chế độ 40 MHz (Ch.4.1)

% --- Nhiễu (khớp công thức lý thuyết Ch.4.2.4) ---
NF     = 7;                          % Noise Figure máy thu điển hình (dB)
N0     = -174 + 10*log10(BW) + NF;  % Nhiễu nhiệt nền tổng hợp (dBm)
                                     % Công thức: N0 = kTB + NF
                                     % Tại BW=40MHz: N0 ≈ -174 + 76 + 7 = -91 dBm
%
% Giả định Kịch bản 1: môi trường không có nhiễu đồng kênh đáng kể
% → I_mean đặt ở mức rất thấp (-110 dBm) để chỉ tính nhiễu nhiệt N0
% → Phù hợp với giả định "I = 0" trong phần mô tả kịch bản (Ch.4.3.1)
I_mean = -110;                       % Nhiễu đồng kênh nền (dBm) — gần như bằng 0

% --- Rician K-factor (Ch.4.2.3) ---
% K = 5: phù hợp UAV bay ở độ cao 50 m, tia LOS chiếm ưu thế (Ch.4.2.3)
% K tăng khi UAV nâng cao hơn (góc ngẩng lớn → LOS mạnh hơn đa đường)
K = 5;

%% =========================
% 2. WIFI MCS TABLE
% =========================
% Bảng MCS chuẩn IEEE 802.11ac, 1 spatial stream, BW = 40 MHz (Ch.4.2.5)
% Thuật toán điều chế tự động (Dynamic Rate Control): chọn MCS cao nhất
% mà SINR hiện tại vẫn đáp ứng ngưỡng SNR_req tương ứng.
% Nguồn: IEEE 802.11ac-2013 Table 22-26

SNR_req  = [4  7  10  13  16  20  24  28];  % Ngưỡng SINR tối thiểu theo từng MCS (dB)
PHY_rate = [13 27 40  54  81 108 121 135];  % Tốc độ PHY tương ứng (Mbps)
%            MCS0 MCS1 MCS2 MCS3 MCS4 MCS5  MCS6  MCS7
%            BPSK QPSK QPSK 16Q  16Q  64Q   64Q  256Q

MAC_eff          = 0.7;      % Hiệu suất tầng MAC (≈70% sau overhead CSMA/CA)
Packet_Size_bits = 1500 * 8; % Kích thước gói Ethernet tiêu chuẩn (bits)

%% =========================
% 3. KHOẢNG CÁCH & CRITICAL DISTANCE — KỊCH BẢN 1
% =========================
% Khoảng cách tới hạn dc = 4*ht*hr/lambda (Ch.4.2.1):
%   dc = 4 × 50 × 2 / 0.06 = 6667 m
%
% Tại d < dc: suy hao theo mô hình Free-Space (–20 dB/decade)
% Tại d > dc: suy hao theo mô hình Two-Ray bậc 4 (–40 dB/decade)
%             do giao thoa triệt tiêu giữa tia LOS và tia phản xạ mặt đất
%
% Dải khảo sát bắt đầu từ 100 m để thể hiện đỉnh Throughput/SINR vùng gần
% → cần thiết để phân tích đủ 3 vùng theo lý thuyết (Ch.4.3.1):
%   Vùng 1 (An toàn)   : d = 100 m – ~3000 m  → SINR cao, Throughput đỉnh
%   Vùng 2 (Suy giảm)  : d = 3000 m – dc      → SINR giảm dần, MCS lùi bậc
%   Vùng 3 (Sụp đổ)    : d > dc = 6667 m      → suy hao bậc 4, kết nối video mất

dc = (4 * ht * hr) / lambda;
fprintf('=== Khoảng cách tới hạn dc = %.0f m ===\n', dc);

d  = 100 : 100 : 12000;   % 100 m – 12 km, bước 100 m
Nd = length(d);

%% =========================
% 4. MONTE CARLO SETUP
% =========================
% Monte Carlo 500 lần lặp: đủ để trung bình hội tụ ổn định
% (sai số chuẩn ~ 1/sqrt(500) ≈ 4.5% so với giá trị kỳ vọng lý thuyết)
Nrun = 500;

Throughput_all = zeros(Nrun, Nd);
SINR_all       = zeros(Nrun, Nd);
Latency_all    = zeros(Nrun, Nd);

%% =========================
% 5. MAIN SIMULATION LOOP — KỊCH BẢN 1
% =========================
% Vòng lặp Monte Carlo: mỗi run mô phỏng một thực hiện ngẫu nhiên của kênh
% (shadowing + fading khác nhau) tại từng khoảng cách d(i).
% Kết quả cuối lấy trung bình qua 500 runs → phản ánh đặc tính thống kê kênh.

fprintf('Bắt đầu mô phỏng Kịch bản 1: Monte Carlo (%d runs × %d distances)...\n', Nrun, Nd);
tic;
for k = 1:Nrun
    for i = 1:Nd

        % ===== Mô hình Two-Ray Ground Reflection (Ch.4.2.1) =====
        % Phân vùng theo khoảng cách tới hạn dc:
        %   d <= dc: công suất thu theo FSPL (Free-Space Path Loss)
        %            PL = 20log(d) + 20log(4π/λ)  [dB]
        %   d >  dc: suy hao tăng thêm 40log(d/dc) so với PL tại dc
        %            tương đương hàm bậc 4 của khoảng cách (–40 dB/decade)
        if d(i) <= dc
            PL = 20*log10(d(i)) + 20*log10(4*pi / lambda);
        else
            PL_dc = 20*log10(dc) + 20*log10(4*pi / lambda);
            PL    = PL_dc + 40*log10(d(i) / dc);
        end
        % Công suất thu cơ sở: Pr_base = Pt + Gt + Gr - PL  [dBm]
        Pr_base = Pt + Gt + Gr - PL;

        % ===== Shadowing Log-Normal (Ch.4.2.2) =====
        % Biến thiên môi trường (địa hình, vật cản) được mô hình hóa
        % bằng phân bố Gauss: X_sigma ~ N(0, sigma²), sigma = 3 dB
        % Nguồn: Rappaport (2002), Wireless Communications, Sec.3.3
        shadowing   = randn * 3;              % N(0, 9): sigma = 3 dB
        Pr_shadowed = Pr_base + shadowing;

        % ===== Rician Fading (Ch.4.2.3) =====
        % Liên kết UAV–GCS luôn có tia LOS → phân bố Rician phù hợp hơn Rayleigh
        % Biên độ kênh: h = sqrt(K/(K+1)) + sqrt(1/(2(K+1))) × (N(0,1)+jN(0,1))
        %   Thành phần xác định: LOS_amp = sqrt(K/(K+1))
        %   Thành phần ngẫu nhiên: NLOS_amp ~ Gaussian phức (đa đường tán xạ)
        % Nguồn: Simon & Alouini (2005), Digital Communication over Fading Channels
        LOS_amp  = sqrt(K / (K + 1));
        NLOS_amp = sqrt(1 / (2*(K+1))) * (randn + 1i*randn);
        fading   = LOS_amp + NLOS_amp;
        % Cộng thêm ảnh hưởng fading (dB): Pr = Pr_shadowed + 20log|h|
        Pr       = Pr_shadowed + 20*log10(abs(fading));

        % ===== Nhiễu & SINR (Ch.4.2.4) =====
        % Nhiễu tức thời: I_inst = I_mean + N(0, 4) — biến động nhỏ quanh nền
        % Tổng công suất nhiễu + nhiệt: NI_mW = N0_mW + I_mW  [mW]
        % SINR = Pr - 10log(NI_mW)  [dB]
        % Vì I_mean = -110 dBm << N0 ≈ -91 dBm → SINR ≈ Pr - N0 (chỉ nhiễu nhiệt)
        I_inst         = I_mean + randn * 2;  % Nhiễu đồng kênh tức thời (dBm)
        NI_mW          = 10^(N0/10) + 10^(I_inst/10);
        SINR           = Pr - 10*log10(NI_mW);
        SINR_all(k, i) = SINR;

        % ===== Adaptive MCS — Điều chế tự động (Ch.4.2.5) =====
        % Chọn MCS cao nhất mà SINR hiện tại vẫn đáp ứng ngưỡng SNR_req
        % → Tối đa hóa thông lượng trong điều kiện kênh truyền hiện tại
        idx = find(SINR >= SNR_req, 1, 'last');
        if isempty(idx)
            rate = 0;   % SINR < 4 dB: không thể giải mã → kết nối sụp đổ
        else
            rate = PHY_rate(idx);
        end

        % ===== Throughput / Goodput (Ch.4.3.1) =====
        % Goodput = PHY_rate × MAC_efficiency
        % MAC_eff = 0.7: phản ánh overhead CSMA/CA, ACK, header 802.11
        % Khi rate = 0 (SINR < ngưỡng MCS0 = 4 dB): không có kết nối → Goodput = 0
        if rate == 0
            tp = 0;
        else
            tp = rate * MAC_eff;
        end
        Throughput_all(k, i) = tp;

        % ===== Latency E2E (Ch.1.3.1) =====
        % Tổng độ trễ đầu cuối = Prop_Delay + Tx_Delay + MAC_Queue_Delay
        %   Prop_Delay      = d/c       (truyền sóng điện từ trong không khí)
        %   Tx_Delay        = L/R       (thời gian truyền gói L bits ở tốc độ R)
        %   MAC_Queue_Delay = 2–5 ms    (CSMA/CA backoff + xếp hàng ngẫu nhiên)
        % Ngưỡng: latency > 200 ms → gán 200 ms (kết nối mất kiểm soát thực tế)
        Prop_Delay      = d(i) / c;
        MAC_Queue_Delay = 0.002 + rand * 0.003;
        if rate > 0
            Tx_Delay = Packet_Size_bits / (rate * 1e6);
        else
            Tx_Delay = inf;
        end

        current_latency = Prop_Delay + Tx_Delay + MAC_Queue_Delay;

        if rate == 0 || ~isfinite(current_latency) || current_latency > 0.2
            Latency_all(k, i) = 200;   % Giới hạn trên: kết nối không dùng được
        else
            Latency_all(k, i) = current_latency * 1000;   % Đổi s → ms
        end
    end
end

elapsed = toc;
fprintf('Kịch bản 1 hoàn thành! Thời gian: %.1f giây.\n', elapsed);

%% =========================
% 6. AVERAGING & SMOOTHING — KỊCH BẢN 1
% =========================
% Lấy trung bình qua 500 runs Monte Carlo → loại bỏ nhiễu thống kê
% Làm mượt bằng moving average (cửa sổ 8 điểm) → đường xu hướng rõ ràng hơn
% cho đồ thị, phù hợp báo cáo kỹ thuật

Throughput_avg = mean(Throughput_all, 1);
SINR_avg       = mean(SINR_all, 1);
Latency_avg    = mean(Latency_all, 1);

win = 8;   % Cửa sổ moving average: 8 điểm × 100 m/điểm = 800 m
Throughput_smooth = movmean(Throughput_avg, win);
SINR_smooth       = movmean(SINR_avg, win);
Latency_smooth    = movmean(Latency_avg, win);

%% =========================
% 7. JITTER MODEL — KỊCH BẢN 2
% =========================
% Kịch bản 2: Cố định khoảng cách d = 2000 m (vùng an toàn, SINR cao)
%             Thay đổi vận tốc v từ 0 → 30 m/s để khảo sát hiệu ứng Doppler
%
% Mô hình Jitter theo hiệu ứng Doppler/ICI (Ch.4.2.7):
%   fd = v × cos(θ) / lambda     [Hz]   — Tần số dịch chuyển Doppler
%   Jitter(ms) = J0 + alpha × fd²       — Jitter tăng theo bậc 2 của fd
%
% Cơ sở vật lý: Khi UAV di chuyển, dịch tần Doppler fd phá vỡ tính trực
% giao của các sóng mang con OFDM → sinh nhiễu liên sóng mang (ICI).
% ICI ∝ fd² → tầng MAC phải truyền lại gói (Retransmission) → Jitter tăng.
%
% Hiệu chỉnh tham số alpha để khớp Ch.4.3.2 (Jitter vượt 30 ms tại v = 7.5 m/s):
%   Góc worst-case: θ = 0° (UAV bay thẳng về GCS → cos(θ) = 1, fd lớn nhất)
%   fd_crit = 7.5 / 0.06 = 125 Hz
%   30 = 5 + alpha × 125²  →  alpha = 25/15625 = 0.0016
%   Kiểm tra: J(7.5) = 5 + 0.0016 × 15625 = 5 + 25 = 30 ms ✓
%
% Lưu ý về tỷ số fd/Δf_sc (IEEE 802.11ac, BW=40MHz, Δf_sc = 312.5 kHz):
%   Tại v = 30 m/s: fd_max = 500 Hz → fd/Δf_sc = 500/312500 ≈ 0.0016 << 1%
%   → ICI thực tế rất nhỏ về mặt vật lý; mô hình fd² là xấp xỉ worst-case
%     để minh họa xu hướng Jitter tăng theo vận tốc (Ch.4.3.2)

theta  = 0;               % Góc giữa hướng bay UAV và LOS đến GCS (rad), θ=0: worst-case
v      = 0 : 0.5 : 30;   % Vận tốc UAV (m/s): 0–30 m/s, bước 0.5 m/s
fd     = v * cos(theta) / lambda;   % Dịch tần Doppler (Hz): fd = v·cos(θ)/λ
J0     = 5;               % Jitter nền tại vận tốc 0 (ms) — do MAC queue bình thường
alpha  = 0.0016;          % Hệ số ICI: hiệu chỉnh để Jitter = 30 ms tại v = 7.5 m/s
Jitter = J0 + alpha * fd.^2;        % Mô hình Jitter tổng hợp (ms)

% Phân tích tỷ số fd/Δf_sc — kiểm tra mức độ ICI thực tế
delta_f_sc = 312.5e3;               % Khoảng cách sóng mang con OFDM (Hz) — 802.11ac 40MHz
fd_ratio   = fd / delta_f_sc;       % Tỷ số vô thứ nguyên
fprintf('\n--- Phân tích Doppler Kịch bản 2 ---\n');
fprintf('Góc θ = %.0f° → cos(θ) = %.2f (worst-case)\n', rad2deg(theta), cos(theta));
fprintf('v_max = 30 m/s → fd_max = %.1f Hz, fd/Δf_sc = %.5f\n', max(fd), max(fd_ratio));
fprintf('→ fd/Δf_sc << 1%% tại mọi v ≤ 30 m/s: ICI vật lý rất nhỏ\n');
fprintf('→ Mô hình J0+alpha·fd² là xấp xỉ worst-case để minh họa xu hướng Jitter\n');

%% =========================
% 8. NGƯỠNG KPI — Bảng 2.1
% =========================
% Các ngưỡng hiệu năng yêu cầu từ Bảng 2.1 (Ch.2.3.2):
%   Throughput ≥ 5 Mbps : đảm bảo luồng video HD H.264/H.265 không bị hạ chất lượng
%   Latency   < 100 ms  : giới hạn sinh lý an toàn cho điều khiển FPV (Ch.1.3.1)
%   Jitter    < 30 ms   : ngăn tràn bộ đệm UDP, tránh giật/xé hình (Ch.1.3.2)

realtime_th = 5;    % Throughput tối thiểu cho video HD (Mbps)
latency_th  = 100;  % Độ trễ E2E tối đa cho FPV/realtime (ms)
jitter_th   = 30;   % Jitter tối đa cho luồng UDP video (ms)

% Xác định điểm vi phạm KPI — Kịch bản 1
fail_idx   = Throughput_smooth < realtime_th;
fail_point = find(fail_idx, 1, 'first');

high_latency_idx = Latency_smooth > latency_th;
lat_fail_point   = find(high_latency_idx, 1, 'first');

% Xác định điểm vi phạm Jitter — Kịch bản 2
jitter_fail_idx = find(Jitter > jitter_th, 1, 'first');

%% =========================
% 9. PLOTS
% =========================
% Màu sắc theo quy ước IEEE (dùng nhất quán trên tất cả các Figure)
col_raw    = [0.70 0.70 0.70];   % Xám nhạt — đường Raw (Monte Carlo)
col_smooth = [0.00 0.45 0.74];   % Xanh dương IEEE — đường Smoothed
col_warn   = [1.00 0.60 0.60];   % Đỏ nhạt — vùng vi phạm KPI

%% ===== Figure 1: Throughput vs Khoảng cách — Kịch bản 1 =====
% Thể hiện 3 vùng hoạt động theo lý thuyết Ch.4.3.1:
%   Vùng AN TOÀN (nền xanh): d < dc, Throughput >> 5 Mbps
%   Vùng SỤP ĐỔ  (nền cam) : d > dc, Throughput < 5 Mbps, suy hao bậc 4
figure('Name','Fig 1: Throughput vs Distance','NumberTitle','off');
hold on; grid on;

ylim_max = max(Throughput_smooth)*1.35 + 5;
patch([d(1) dc dc d(1)], [0 0 ylim_max ylim_max], [0.9 1.0 0.9], ...
    'EdgeColor','none','FaceAlpha',0.3,'DisplayName','Vùng An toàn (d < d_c)');
patch([dc dc d(end) d(end)], [0 0 ylim_max ylim_max], [1.0 0.95 0.85], ...
    'EdgeColor','none','FaceAlpha',0.3,'DisplayName','Vùng Suy giảm / Sụp đổ (d > d_c)');

plot(d, Throughput_avg,    '--','Color',col_raw,    'DisplayName','Raw (Monte Carlo)');
plot(d, Throughput_smooth, '-', 'Color',col_smooth, 'LineWidth',2,'DisplayName','Smoothed');
yline(realtime_th,'r--','LineWidth',1.5,'Label','Ngưỡng KPI Video ≥ 5 Mbps (Bảng 2.1)');

xline(dc,'k:','LineWidth',1.5,'Label',sprintf('d_c = %.0f m',dc), ...
    'LabelVerticalAlignment','bottom');

if ~isempty(fail_point)
    x_f = d(fail_point); y_f = Throughput_smooth(fail_point);
    area(d(fail_idx), Throughput_smooth(fail_idx), ...
        'FaceColor',col_warn,'EdgeColor','none','FaceAlpha',0.5, ...
        'DisplayName','Dưới ngưỡng KPI (< 5 Mbps)');
    plot(x_f, y_f,'ro','MarkerSize',8,'LineWidth',2,'HandleVisibility','off');
    text(x_f+100, y_f+3, sprintf('Sụp đổ @ %d m',round(x_f)), ...
        'Color','r','FontWeight','bold','FontSize',9);
end

xlabel('Khoảng cách UAV – GCS (m)','FontSize',11);
ylabel('Throughput MAC (Mbps)','FontSize',11);
title('Throughput vs Khoảng cách – Kịch bản 1 (Ch.4.3.1)','FontSize',12);
legend('Location','northeast','FontSize',9);
xlim([d(1) d(end)]); ylim([0 ylim_max]);

text(dc*0.35, ylim_max*0.92,'Vùng AN TOÀN', ...
    'Color',[0.1 0.5 0.1],'FontWeight','bold','HorizontalAlignment','center','FontSize',9);
text(dc*1.35, ylim_max*0.92,'Vùng SỤP ĐỔ', ...
    'Color',[0.7 0.2 0.0],'FontWeight','bold','HorizontalAlignment','center','FontSize',9);

%% ===== Figure 2: SINR vs Khoảng cách — Kịch bản 1 =====
% Thể hiện sự suy giảm SINR theo khoảng cách và điểm gãy tại dc
% Các đường kẻ ngang là ngưỡng SINR tối thiểu của từng mức MCS (Ch.4.2.5)
figure('Name','Fig 2: SINR vs Distance','NumberTitle','off');
hold on; grid on;

ylim_sinr = [min(SINR_smooth)-5, max(SINR_smooth)+5];
patch([d(1) dc dc d(1)], [ylim_sinr(1) ylim_sinr(1) ylim_sinr(2) ylim_sinr(2)], ...
    [0.9 1.0 0.9],'EdgeColor','none','FaceAlpha',0.3,'DisplayName','Vùng An toàn');
patch([dc dc d(end) d(end)], [ylim_sinr(1) ylim_sinr(1) ylim_sinr(2) ylim_sinr(2)], ...
    [1.0 0.95 0.85],'EdgeColor','none','FaceAlpha',0.3,'DisplayName','Vùng Suy giảm / Sụp đổ');

plot(d, SINR_avg,    '--','Color',col_raw,    'DisplayName','Raw');
plot(d, SINR_smooth, '-', 'Color',col_smooth, 'LineWidth',2,'DisplayName','Smoothed');

% Vẽ ngưỡng SINR tối thiểu của từng MCS (Ch.4.2.5)
mcs_col = lines(length(SNR_req));
for m = 1:length(SNR_req)
    yline(SNR_req(m),':','Color',mcs_col(m,:),'Alpha',0.7, ...
        'Label',sprintf('MCS%d (%d dB)',m-1,SNR_req(m)), ...
        'LabelHorizontalAlignment','right','FontSize',7);
end

xline(dc,'k:','LineWidth',1.5,'Label',sprintf('d_c = %.0f m',dc), ...
    'LabelVerticalAlignment','bottom');

% Tô vùng SINR dưới ngưỡng MCS0 = 4 dB — kết nối mất hoàn toàn
weak_idx = SINR_smooth < SNR_req(1);
if any(weak_idx)
    area(d(weak_idx), SINR_smooth(weak_idx), ...
        'FaceColor',col_warn,'EdgeColor','none','FaceAlpha',0.5,'BaseValue',ylim_sinr(1), ...
        'DisplayName','Vùng tín hiệu yếu (SINR < 4 dB)');
    ws = find(weak_idx,1,'first');
    text(d(ws)+200, SINR_smooth(ws)+2,'Weak Signal Region', ...
        'Color','r','FontWeight','bold','FontSize',9);
end

xlabel('Khoảng cách UAV – GCS (m)','FontSize',11);
ylabel('SINR (dB)','FontSize',11);
title('SINR vs Khoảng cách – Kịch bản 1 (Ch.4.3.1, Mô hình Ch.4.2.1–4.2.4)','FontSize',12);
legend('Location','northeast','FontSize',9);
xlim([d(1) d(end)]); ylim(ylim_sinr);

%% ===== Figure 3: Latency vs Khoảng cách — Kịch bản 1 =====
% Thể hiện độ trễ E2E theo khoảng cách
% Ngưỡng 100 ms: giới hạn sinh lý an toàn cho điều khiển FPV (Ch.1.3.1)
% Vượt ngưỡng này phi công mất khả năng phản ứng kịp thời với chướng ngại vật
figure('Name','Fig 3: Latency vs Distance','NumberTitle','off');
hold on; grid on;

patch([d(1) dc dc d(1)], [0 0 220 220], [0.9 1.0 0.9], ...
    'EdgeColor','none','FaceAlpha',0.3,'DisplayName','Vùng An toàn');
patch([dc dc d(end) d(end)], [0 0 220 220], [1.0 0.95 0.85], ...
    'EdgeColor','none','FaceAlpha',0.3,'DisplayName','Vùng Suy giảm / Sụp đổ');

plot(d, Latency_avg,    '--','Color',col_raw,'DisplayName','Raw');
plot(d, Latency_smooth, '-m','LineWidth',2,  'DisplayName','Smoothed');
yline(latency_th,'r--','LineWidth',1.5, ...
    'Label',sprintf('Ngưỡng FPV Realtime (%d ms) — Ch.1.3.1',latency_th));

xline(dc,'k:','LineWidth',1.5,'Label',sprintf('d_c = %.0f m',dc), ...
    'LabelVerticalAlignment','bottom');

if any(high_latency_idx)
    area(d(high_latency_idx), Latency_smooth(high_latency_idx), ...
        'FaceColor',col_warn,'EdgeColor','none','FaceAlpha',0.5, ...
        'BaseValue',latency_th,'DisplayName','Vượt ngưỡng Latency (> 100 ms)');
    if ~isempty(lat_fail_point)
        x_lf = d(lat_fail_point); y_lf = Latency_smooth(lat_fail_point);
        plot(x_lf, y_lf,'ro','MarkerSize',8,'LineWidth',2,'HandleVisibility','off');
        text(x_lf+150, y_lf+8, sprintf('Latency Fail\n@ %d m',round(x_lf)), ...
            'Color','r','FontWeight','bold','FontSize',9);
    end
end

xlabel('Khoảng cách UAV – GCS (m)','FontSize',11);
ylabel('Độ trễ E2E (ms)','FontSize',11);
title('Độ trễ End-to-End vs Khoảng cách – Kịch bản 1 (Ch.1.3.1)','FontSize',12);
legend('Location','northwest','FontSize',9);
xlim([d(1) d(end)]); ylim([0 220]);

%% ===== Figure 4: Jitter vs Vận tốc UAV — Kịch bản 2 =====
% Thể hiện 3 vùng hoạt động theo lý thuyết Ch.4.3.2:
%   Vùng ổn định    : v = 0–7.5 m/s  → Jitter < 30 ms, video mượt
%   Điểm tới hạn    : v ≈ 7.5 m/s (27 km/h) → Jitter = 30 ms, điểm gãy
%   Vùng mất ổn định: v > 7.5 m/s   → Jitter > 30 ms, video giật/xé hình
figure('Name','Fig 4: Jitter vs UAV Velocity','NumberTitle','off');
hold on; grid on;

% Tô vùng Jitter vượt ngưỡng (Unstable Video)
fail_v = Jitter > jitter_th;
if any(fail_v)
    area(v(fail_v), Jitter(fail_v), ...
        'FaceColor',col_warn,'EdgeColor','none','FaceAlpha',0.5, ...
        'BaseValue',jitter_th,'DisplayName','Vùng mất ổn định (Jitter > 30 ms)');
end

plot(v, Jitter,'b','LineWidth',2,'DisplayName', ...
    sprintf('Jitter = %d + %.4f × f_d² (ms)',J0,alpha));
yline(jitter_th,'r--','LineWidth',1.5, ...
    'Label',sprintf('Ngưỡng KPI Jitter (%d ms) — Bảng 2.1',jitter_th));

% Đánh dấu điểm vận tốc tới hạn
if ~isempty(jitter_fail_idx)
    v_crit  = v(jitter_fail_idx);
    fd_crit = fd(jitter_fail_idx);
    plot(v_crit, Jitter(jitter_fail_idx),'ro','MarkerSize',8,'LineWidth',2, ...
        'HandleVisibility','off');
    text(v_crit+0.5, Jitter(jitter_fail_idx)+3, ...
        sprintf('ICI vượt ngưỡng\nv = %.1f m/s (%.0f km/h)\nf_d = %.0f Hz', ...
        v_crit, v_crit*3.6, fd_crit), ...
        'Color','r','FontWeight','bold','FontSize',9);
    text(mean(v(fail_v)), max(Jitter)*0.75,'Unstable Video', ...
        'Color','r','FontWeight','bold','HorizontalAlignment','center','FontSize',10);
end

% Ghi chú tần số Doppler tại các mốc vận tốc
for ii = 1:3:length(v)
    text(v(ii), Jitter(ii)-4, sprintf('f_d\n%.0f Hz',fd(ii)), ...
        'FontSize',7,'HorizontalAlignment','center','Color',[0.4 0.4 0.4]);
end

xlabel('Vận tốc UAV (m/s)','FontSize',11);
ylabel('Jitter (ms)','FontSize',11);
title('Jitter vs Vận tốc UAV – Kịch bản 2 (Ch.4.3.2, Mô hình Ch.4.2.7)','FontSize',12);
legend('Location','northwest','FontSize',9);
xlim([v(1) v(end)]); ylim([0 max(Jitter)*1.2+5]);

%% =========================
% 10. SUMMARY REPORT
% =========================
fprintf('\n============================================================\n');
fprintf('         TÓM TẮT KẾT QUẢ MÔ PHỎNG HAI KỊCH BẢN\n');
fprintf('============================================================\n');
fprintf('--- Thông số hệ thống ---\n');
fprintf('Tần số              : %.1f GHz\n', fc/1e9);
fprintf('Băng thông          : %.0f MHz\n', BW/1e6);
fprintf('Công suất phát UAV  : %.0f dBm\n', Pt);
fprintf('Gain anten GCS      : %.0f dBi\n', Gr);
fprintf('Độ cao UAV / GCS    : %.0f m / %.0f m\n', ht, hr);
fprintf('Khoảng cách tới hạn : %.0f m (Two-Ray d_c = 4×ht×hr/λ)\n', dc);
fprintf('Rician K-factor     : %d\n', K);
fprintf('Monte Carlo runs    : %d\n', Nrun);
fprintf('------------------------------------------------------------\n');
fprintf('--- Kết quả Kịch bản 1: Throughput / SINR / Latency vs Khoảng cách ---\n');
if ~isempty(fail_point)
    fprintf('[KQ1] Throughput sụp đổ    : @ %.0f m (xuống dưới %.0f Mbps)\n', ...
        d(fail_point), realtime_th);
else
    fprintf('[KQ1] Throughput           : Không sụp đổ trong dải khảo sát 100–12000 m\n');
end
fprintf('[KQ2] SINR max / min       : %.1f dB / %.1f dB\n', ...
    max(SINR_smooth), min(SINR_smooth));
if ~isempty(lat_fail_point)
    fprintf('[KQ3] Latency vượt ngưỡng : @ %.0f m (vượt %.0f ms)\n', ...
        d(lat_fail_point), latency_th);
else
    fprintf('[KQ3] Latency             : Không vượt ngưỡng trong dải khảo sát\n');
end
fprintf('------------------------------------------------------------\n');
fprintf('--- Kết quả Kịch bản 2: Jitter vs Vận tốc UAV ---\n');
if ~isempty(jitter_fail_idx)
    fprintf('[KQ4] Jitter vượt ngưỡng  : v > %.1f m/s (%.0f km/h), f_d = %.0f Hz\n', ...
        v(jitter_fail_idx), v(jitter_fail_idx)*3.6, fd(jitter_fail_idx));
    fprintf('      fd/Δf_sc tại điểm tới hạn : %.5f (ICI vật lý rất nhỏ)\n', ...
        fd(jitter_fail_idx)/delta_f_sc);
else
    fprintf('[KQ4] Jitter              : Luôn trong ngưỡng cho v = 0–30 m/s\n');
end
fprintf('============================================================\n');
fprintf('→ [Kịch bản 1] UAV hoạt động AN TOÀN trong bán kính < d_c = %.0f m\n', dc);
if ~isempty(jitter_fail_idx)
    v_safe = v(jitter_fail_idx) - 0.5;
    fprintf('→ [Kịch bản 2] Vận tốc tối đa để video ổn định : %.1f m/s (%.0f km/h)\n', ...
        v_safe, v_safe*3.6);
    fprintf('   (Góc θ = %.0f° — worst-case: UAV bay thẳng về GCS)\n', rad2deg(theta));
else
    fprintf('→ [Kịch bản 2] Jitter ổn định toàn bộ dải vận tốc 0–30 m/s\n');
end
fprintf('============================================================\n');