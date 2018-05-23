
% Bu fonksiyon, üç motorun deðerleriyle birlikte robotun TCP'sinin son konumunu
% hesaplamaktadýr. "theta(n)", "(n)" linkini döndüren motoru temsil
% etmektedir. "P", TCP'nin "x", "y" ve "z" düzlemlerindeki son kartezyen koordinatlarýný 
% temsil etmektedir.

function [ P ] = Forward_Kinematics_3DoF( degtheta )

%% ROBOT PARAMETRELERÝ
% Robot yapýsýnýn deðerleri.

% Ýlk çubuk parametreleri
alpha1 = 90; % derece cinsinden
a1 = 0; % mm cinsinden
d1 = 17.5; % mm cinsinden

% Ýkinci çubuk parametreleri
alpha2 = 0; % derece cinsinden
a2 = 65; % mm cinsinden
d2 = 0; % mm cinsinden

% Üçüncü çubuk parametreleri
alpha3 = 0; % derece cinsinden
a3 = 67.5; % mm cinsinden
d3 = 0; % mm cinsinden

% Motorlarýn açýlarý
theta1 = degtheta(1);
theta2 = degtheta(2);
theta3 = degtheta(3);

%% LÝNKLERÝN MATRÝSLERÝ
% Robotun her bir çubuðunun tanýmlanmasý amacýyla Denavit-Hartenberg parametreleri kullanýlarak
% oluþturulmuþ matrisler.

T01 = [ cosd(theta1)  -sind(theta1)*cosd(alpha1)   sind(theta1)*sind(alpha1)  a1*cosd(theta1) ;
        sind(theta1)   cosd(theta1)*cosd(alpha1)  -cosd(theta1)*sind(alpha1)  a1*sind(theta1) ;
             0               sind(alpha1)                cosd(alpha1)               d1        ;
             0                    0                           0                      1        ];

T12 = [ cosd(theta2)  -sind(theta2)*cosd(alpha2)   sind(theta2)*sind(alpha2)  a2*cosd(theta2) ;
        sind(theta2)   cosd(theta2)*cosd(alpha2)  -cosd(theta2)*sind(alpha2)  a2*sind(theta2) ;
             0               sind(alpha2)                cosd(alpha2)               d2        ;
             0                    0                           0                      1        ];
 
T23 = [ cosd(theta3)  -sind(theta3)*cosd(alpha3)   sind(theta3)*sind(alpha3)  a3*cosd(theta3) ;
        sind(theta3)   cosd(theta3)*cosd(alpha3)  -cosd(theta3)*sind(alpha3)  a3*sind(theta3) ;
             0               sind(alpha3)                cosd(alpha3)               d3        ;
             0                    0                           0                      1        ];         

%% ÝLERÝ KÝNEMATÝK   
         
T03 = T01*T12*T23;

%% HESAPLANAN KONUM
% Son TCP konumu

P(1) = T03(1,4); % x ekseni boyunca, mm cinsinden
P(2) = T03(2,4); % y ekseni boyunca, mm cinsinden
P(3) = T03(3,4); % z ekseni boyunca, mm cinsinden