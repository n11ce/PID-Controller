%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% 1 SORU CEVAPLARI %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



a = 0.0414;
b = 0.441;
e = 0.288;
f = 1;
c = 0.144;

numerator = c;
% Transfer fonksiyonunu tanımladık
denominator = [a*e, (a*f + b*e),(b*f + c^2),0];
sys = tf(numerator, denominator);
% Kökler
roots_of_tf = roots(denominator);

% Kökleri kontrol için ekrana yazdırdık
disp('Transfer fonksiyonunun kökleri:');
disp(roots_of_tf);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%    2-3 SORU CEVAPLARI %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Kök lokusu çizimi % periyot ve kazanç görmek için gerekli
figure;
rlocus(sys);
title('Kök Lokus Grafiği');
grid on;

gain = 2.48; %% imajinel köklerin kesiştiği yerden kazanci çektik

% Seçilen kazanç ile yeni sistem
new_sys = feedback(gain * sys, 1);
figure;
step(new_sys);
title('Adım Tepkisi');

%%% MAKSİMUM PERYİOT VE KAZANÇ
Ku = 45.3; % mak kazanç
Wu = 6.22;
Pu = (2*3.14)/Wu; % mak peryot

t = 0:0.001:5; % toplam süre 5 saniye örnekleme zaman aralığı 0.001 sn

% Sadece P(s) Sistem Tasarımı

figure;
step(new_sys,t);
title('P(s) Adım Tepkisi');
info_pid = stepinfo(new_sys);
disp('P(s) Sistemin Performans Ölçümleri:');
disp(info_pid);
%%% Kararlı hata değeri
[y, t] = step(new_sys, 5);
steady_state_value = y(end);
reference_value = 1; % Adım girişi için referans değer
steady_state_error = reference_value - steady_state_value;

disp('Kararlı Hal Hatası:');
disp(steady_state_error);

% Sadece P(s) ve P Sistem Tasarımı
Kp = Ku/2;

figure;
PID_Cont=pid(Kp);
OpenSys=series(PID_Cont,sys);
CloseSys=feedback(OpenSys,1);
step(CloseSys,t);
title('P(s) ve P Tepkisi');
info_pid = stepinfo(CloseSys);
disp('P Kontrolcü ve P(s) Sistemin Performans Ölçümleri:');
disp(info_pid);

%%% Kararlı hata değeri
[y, t] = step(CloseSys, 5);
steady_state_value = y(end);
reference_value = 1; % Adım girişi için referans değer
steady_state_error = reference_value - steady_state_value;

disp('Kararlı Hal Hatası:');
disp(steady_state_error);

% Sadece P(s) ve PI Sistem Tasarımı
Kp = Ku/2.2;
Ki=Kp/(Pu/1.2);

PID_Cont=pid(Kp,Ki);
OpenSys=series(PID_Cont,sys);
CloseSys=feedback(OpenSys,1);
figure;
step(CloseSys,t);
title('P(s) ve PI Tepkisi');
info_pid = stepinfo(CloseSys);
disp('PI Kontrolcü ve P(s) Sistemin Performans Ölçümleri:');
disp(info_pid);

[y, t] = step(CloseSys, 5);
steady_state_value = y(end);
reference_value = 1; % Adım girişi için referans değer
steady_state_error = reference_value - steady_state_value;

disp('Kararlı Hal Hatası:');
disp(steady_state_error);

% P(s) ve PID Sistem Tasarımı
Kp = Ku/1.7;
Ki=Kp/(Pu/2);
Kd=Kp*(Pu/8);

PID_Cont=pid(Kp,Ki,Kd);
OpenSys=series(PID_Cont,sys);
CloseSys=feedback(OpenSys,1);
figure;
step(CloseSys,t);
title('P(s) ve PID Tepkisi');
info_pid = stepinfo(CloseSys);
disp('PID Kontrolcü ve P(s) Sistemin Performans Ölçümleri:');
disp(info_pid);

[y, t] = step(CloseSys, 5);
steady_state_value = y(end);
reference_value = 1; % Adım girişi için referans değer
steady_state_error = reference_value - steady_state_value;

disp('Kararlı Hal Hatası:');
disp(steady_state_error);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% 4 SORU CEVAPLARI %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Kp = Kp * 0.7;
Ki = Ki * 0.25;
Kd = Kd * 2.5; 

% Bu değerlerle
% RiseTime: 0.1926
% Overshoot: 6.8181
%Kararlı Hal Hatası: 8.9299e-04 (tolere edilebilecek kadar düşük)

% yeni sistemi oluşturuyoruz
pidcontroller = pid(Kp, Ki, Kd);
opensys = series(pidcontroller, sys);
closesys = feedback(opensys, 1);
    
figure;
step(closesys, 5);
title('PID ile Kapalı Döngü Adım Tepkisi - İnce Ayar');
grid on;
    
info = stepinfo(closesys);
disp('İnce Ayar Sonrası PID Adım Tepkisi Performans Ölçümleri:');
disp(info);
 
%%% Kararlı hata değeri
[y, t] = step(closesys, 5);
steady_state_value = y(end);
reference_value = 1; % Adım girişi için referans değer
steady_state_error = reference_value - steady_state_value;

disp('Kararlı Hal Hatası:');
disp(steady_state_error);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% 5 SORU CEVAPLARI %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Bozucu sinyal
disturbance = tf(1, [1 0]);

% P Kontrolör
Kp = Ku/2;
P_controller = pid(Kp, 0, 0);
P_sys = feedback(P_controller * sys, 1);
P_dist_response = feedback(sys, P_controller);
figure;
step(P_dist_response * disturbance);
title('P Kontrolör ile Adım Bozucu Tepkisi');
grid on;

% PI Kontrolör
Kp = Ku/2.2;
Ki=Kp/(Pu/1.2);
PI_controller = pid(Kp, Ki);
PI_sys = feedback(PI_controller * sys, 1);
PI_dist_response = feedback(sys, PI_controller);
figure;
step(PI_dist_response * disturbance);
title('PI Kontrolör ile Adım Bozucu Tepkisi');
grid on;

% PID Kontrolör
Kp = Ku/1.7;
Ki=Kp/(Pu/2);
Kd=Kp*(Pu/8);
PID_controller = pid(Kp, Ki, Kd);
PID_sys = feedback(PID_controller * sys, 1);
PID_dist_response = feedback(sys, PID_controller);
figure;
step(PID_dist_response * disturbance);
title('PID Kontrolör ile Adım Bozucu Tepkisi');
grid on;

% Performans Ölçümleri
P_info = stepinfo(P_dist_response * disturbance);
PI_info = stepinfo(PI_dist_response * disturbance);
PID_info = stepinfo(PID_dist_response * disturbance);

% Sonuçlar
disp('P Kontrolör Performans Ölçümleri:');
disp(P_info);

disp('PI Kontrolör Performans Ölçümleri:');
disp(PI_info);

disp('PID Kontrolör Performans Ölçümleri:');
disp(PID_info);