clear all
clc 

%---> PROJETO CONTROLADOR DO CONVERSOR INVERSOR <---%

%--- ENTRADAS ---%
fsw = 5e3;                      %Frequência de chaveamento
Tsw = 1/fsw;                    %Período de chaveamento
Lf = (0.0012+2.8949e-04);       %Soma das indutâncias do filtro LCL
Rf = 30e-3;                     %Soma das resistências do filtro LCL
C = 3060e-6;                    %Capacitor do barramento CC
Vdc = 1500;                     %Tensão do barramento CC
Vd = (800/sqrt(3))*sqrt(2);     %Componente d tensão da rede   
w = 2*pi*60;                    %Velocidade angular da rede
wsw = 2*pi*fsw;                 %Freq. chaveamento em radianos
Lb = 5.5488e-3;                 %Indutância do Boost
Rb = 0.1;                       %Resistência do indutor do Boost
Cin = 108.7076e-6;              %Capacitor de entrada do Boost
Rc = 0.01;                      %Resistência do capacitor

inv_Z = tf(1,[1 0], Tsw);
S_discr = (2/Tsw)*((1-inv_Z)/(1+inv_Z)); %Euler regressivo -> Ypes (1 - inv_Z)/Tsw
opt = c2dOptions('Method','zoh');

%Filtro (planta)
numG = [1];
denG = [Lf Rf];
G = tf(numG,denG);

%--- CÁLCULO DOS GANHOS DO CONTROLADOR PI DA MALHA DE CORRENTE ---%
wc = wsw/21.3; 
Kp_i_disc = wc*Lf;
Ki_i_disc = wc*Rf;

%Discretização do controle e da planta
PIc_disc = (Kp_i_disc + (Ki_i_disc/S_discr));
G_planta_I_disc = c2d(G,Tsw,opt);
G_PWM = inv_Z;
Gc_disc = feedback(PIc_disc*G_planta_I_disc*G_PWM,1);

%--- CÁLCULO DOS GANHOS DO CONTROLADOR PI DA MALHA DE TENSÃO ---%
Gt = (3/2)*(Vd/Vdc);
ksi_vdc = 4;
wvdc = wc/38.2;
Kp_vdc_disc = (2*ksi_vdc*wvdc*C)/Gt;
Ki_vdc_disc = ((wvdc)^2*C)/Gt;

G_dc = tf([1],[C 0]);

%Discretização do controle e da planta
PI_vdc_disc = (Kp_vdc_disc + (Ki_vdc_disc/S_discr));
G_vdc_disc = c2d(G_dc,Tsw,opt);
G_vdc_disc = feedback(PI_vdc_disc*Gt*G_vdc_disc,1);

%--- CÁLCULO DOS GANHOS DO CONTROLADOR PI DA MALHA DE REATIVO ---%
H = (-3/2)*Vd;
wq = wvdc*8;
Ki_q_disc = wq/H;
Kp_q_disc = Ki_q_disc*(Lf/Kp_i_disc);

%Discretização do controle e da planta
PI_q_disc = (Kp_q_disc + (Ki_q_disc/S_discr));
G_q_disc = feedback(PI_q_disc*Gc_disc*H,1);

%--- CÁLCULO DOS GANHOS DO CONTROLADOR PI DA MALHA DE CORRENTE DO BOOST ---%
numG_b = [(Vdc)];
denG_b = [(Lb) (Rb)];
G_b = tf(numG_b,denG_b); 

wc_b = wsw/21.3; 
Kp_i_disc_b = wc_b*Lb/Vdc;
Ki_i_disc_b = wc_b*Rb/Vdc;

PIc_disc_b = (Kp_i_disc_b + (Ki_i_disc_b/S_discr));
G_planta_I_disc_b = c2d(G_b,Tsw,opt);
Gc_disc_b = feedback(PIc_disc_b*G_planta_I_disc_b*G_PWM,1);

%--- CÁLCULO DOS GANHOS DO CONTROLADOR PI DA MALHA DE TENSÃO DO BOOST ---%
Vmpp = 1316; 
Impp = 58.54;
Req = Vmpp/Impp;

numGv_b = [-1];
denGv_b = [Cin (1/Req)];
Gv_b = tf(numGv_b,denGv_b); 

wc_bv = wc_b/4.5; 
Kp_v_disc_b = wc_bv*Cin;
Ki_v_disc_b = wc_bv/Req;

PIv_disc_b = (Kp_v_disc_b + (Ki_v_disc_b/S_discr));
G_planta_V_disc_b = c2d(Gv_b,Tsw,opt);
Gv_disc_b = feedback(PIv_disc_b*G_planta_V_disc_b,1);

%--- ANÁLISE DINÂMICA ---%
figure
bode(Gc_disc);
hold on
bode(G_vdc_disc);
bode(G_q_disc);
% figure
% step(Gc_disc);
% hold on
% step(G_vdc_disc);
% step(G_q_disc);
% figure
% bode(Gc_disc_b);
% hold on
% bode(Gv_disc_b);
% figure
% step(Gc_disc_b);
% hold on
% step(Gv_disc_b);