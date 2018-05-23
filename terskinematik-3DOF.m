
% Bu fonksiyon son TCP konumuna göre robotun üç motoru için deðerler
% hesaplamaktadýr. "P"deki "x", "y" ve "z" olmak üzere üç koordinatý
% mm cinsinden almakta ve motor açýlarýný derece cinsinden hesaplamaktadýr.
% "theta(n)", "(n)" linkini döndüren motoru temsil etmektedir. 

function [ degtheta ] = Inverse_Kinematics_3DoF( P )

%% ROBOT PARAMETRELERÝ
% Robot yapýsýnýn deðerleri

% Ýlk çubuk parametreleri
syms alpha1 a1 d1 theta1
alpha1 = 90; % derece cinsinden
a1 = 0; % mm cinsinden
d1 = 17.5; % mm cinsinden

% Ýkinci çubuk parametreleri
syms alpha2 a2 d2 theta2
alpha2 = 0; % derece cinsinden
a2 = 65; % mm cinsinden
d2 = 0; % mm cinsinden

% Üçüncü çubuk parametreleri
syms alpha3 a3 d3 theta3
alpha3 = 0; % derece cinsinden
a3 = 67.5; % mm cinsinden
d3 = 0; % mm cinsinden

% Inverse parametreleri
syms r11 r12 r13 r21 r22 r23 r31 r32 r33 Px Py Pz
Px = P(1);
Py = P(2);
Pz = P(3);

%% LÝNKLERÝN MATRÝSLERÝ
% Robotun her çubuðunu tanýmalayabilmek için Denavit-Hartenberg
% parametreleri kullanýlarak oluþturulmuþ matrisler

T01 = [ cos(theta1)  -sin(theta1)*cosd(alpha1)   sin(theta1)*sind(alpha1)  a1*cos(theta1) ;
        sin(theta1)   cos(theta1)*cosd(alpha1)  -cos(theta1)*sind(alpha1)  a1*sin(theta1) ;
             0              sind(alpha1)               cosd(alpha1)             d1        ;
             0                   0                          0                    1        ];

T12 = [ cos(theta2)  -sin(theta2)*cosd(alpha2)   sin(theta2)*sind(alpha2)  a2*cos(theta2) ;
        sin(theta2)   cos(theta2)*cosd(alpha2)  -cos(theta2)*sind(alpha2)  a2*sin(theta2) ;
             0              sind(alpha2)               cosd(alpha2)             d2        ;
             0                   0                          0                    1        ];

T23 = [ cos(theta3)  -sin(theta3)*cosd(alpha3)   sin(theta3)*sind(alpha3)  a3*cos(theta3) ;
        sin(theta3)   cos(theta3)*cosd(alpha3)  -cos(theta3)*sind(alpha3)  a3*sin(theta3) ;
             0              sind(alpha3)               cosd(alpha3)             d3        ;
             0                   0                          0                    1        ];  

%% TERS KÝNEMATÝK

T03 = [ r11  r12  r13  Px  ;
        r21  r22  r23  Py  ;
        r31  r32  r33  Pz  ;
         0    0    0    1  ];

izq = DH_Inverse( T01 ) * T03;
der = T12 * T23;

eqn1 = izq(1,4) == der(1,4);
eqn2 = izq(2,4) == der(2,4);
eqn3 = izq(3,4) == der(3,4);

sol = solve([eqn1, eqn2, eqn3], [theta1, theta2, theta3]);
                      
%% HESAPLANAN AÇILAR
% Her motorun derece cinsinden açýlarý

degtheta(1) = rad2deg(real(double(sol.theta1)));
degtheta(2) = rad2deg(real(double(sol.theta2)));
degtheta(3) = rad2deg(real(double(sol.theta3)));