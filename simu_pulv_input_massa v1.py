import numpy as np
import math
import pandas as pd
# from CUSTOS_ELETRICO import custos
import plotly.graph_objects as go
 
#INPUTS PRINCIPAIS ============================================================================================#  

horas_maximas = 5 #[h]
n_motores = 8 # Número motores
Diametro = 54 # [in]
eta_escmotor = 0.848 # Eficiência ESC e Motor
eta_helice = 0.7719 # Eficiência Hélice
rho = 1.225 # [g/cm3]
Taxa = 10.0 # [L/ha]      
v_pulv = 7 # [m/s] 
v_deslocamento = 10 # [m/s] 
faixa = 10 # [m] 
celulas = 14 # [m/s] 
cap_bat = 30000*0.804 # [m/Ah]*útil                                                                                                     
M_vazio = 38 # [kg] 
M_bat = 12.9 # [kg] 
COAXIAL_80 = 1.397542375147 # Sobressalência de potência do coaxial
fator_erro_otimização = 1.1 # Fator para adequar a otimização
OTIMIZAR_TANQUE = "NAO" #SIM ou NAO para otimizar
bateria_limite = 0.29 # [%] de bateria para RTL BAT
z_deslocando = 14 # [m] 
z_pulverizando = 5.001 # [m] 
acel = 1.4 # [m/s2] 
v_subida = 2 # [m/s] 
omega = 26.0 # [rad/s]
x0 = 42 # [m] Posição Inicial
yi = 37.6+5 # [m] Posição Inicial
zi = 5.0 # [m] Altura pulverização referência
Y = 300 # [m] Lado do Talhão
area_total = 17.28 # [ha] 
X0 = area_total*10000/Y # [m] Lado do Talhão

#INPUTS DO TALHÃO ============================================================================================#  

perna_rtw = [8,15,22,29,35,42,48,53] # Pernas para x voos 
X_rtw =     [x0 + faixa/2 + faixa*(perna_rtw[0]-1),
             x0 + faixa/2 + faixa*(perna_rtw[1]-1),
             x0 + faixa/2 + faixa*(perna_rtw[2]-1),
             x0 + faixa/2 + faixa*(perna_rtw[3]-1),
             x0 + faixa/2 + faixa*(perna_rtw[4]-1),
             x0 + faixa/2 + faixa*(perna_rtw[5]-1),
             x0 + faixa/2 + faixa*(perna_rtw[6]-1),
             x0 + faixa/2 + faixa*(perna_rtw[7]-1)] # [m] para x voos 
Y_rtw =     [293.20, 127.22, 298.32, 240.32, 130.53, 317.62, 298.16, 209.30] # [m] para x voos 
Dist_ensaio_voo = [2511, 2662, 2940, 2817, 2933, 3168, 3071, 2904] # [m] para x voos 
massa_joao = [35.00,29.50,31.50,31.00,30.00,30.00,30.00,26.50] # [L] para x voos 

#CÁLCULOS INICIAIS ============================================================================================# 

it = 0
E_bat_max = (cap_bat/1000)*3.7*celulas
vazao = Taxa/10000 * (v_pulv*60*faixa)
A = np.pi*(0.5*Diametro*0.0254)**2
cnst = 1/(eta_escmotor*eta_helice)
tanque = 0
Y0 = X0
v_yaw = math.pi/180*omega*faixa/2

M_pulv_min = 35
M_pulv_lim = M_pulv_min
M_pulv_max = M_pulv_min 
delta_pulv = 1
faixa_min = faixa; faixa_max = faixa
faixas = np.arange(faixa_min,faixa_max+0.1,1)
volume_tanque = np.arange(M_pulv_min,M_pulv_lim+0.1,delta_pulv)
produtividade_matriz = np.zeros((len(faixas),len(volume_tanque)))
capex_matriz = np.zeros((len(faixas),len(volume_tanque)))
n_passada = 0
area_pulv_local = [0]
talhao_maximus = []
resultados = []
voo_vector = []
dias = []
tempo_manobra = []
EOC_hr = []
vol_conb_consumido = []
tempo_missao = []
talhao_maximus = []
voo_vector = []
produtividade = []
M_pulvs = []
MTOW = []
capacidade_vector = []
vol_comb = []
dist_percorrida = []
dist_pulverizando = []
M_pulv_max2 = []
vazao_bombas = []
EOC_km = []
area_por_voo = []
faixa_vector = []
tempo_por_voo = []
tempo_util = []
drone_e_bateria = []
abastecimentos = []
indice_voo = []
capex = []
T_hover = []; PWM_hover = []; 
T_M1 = []; PWM_M1 = []; ef_M1 = []; Preq_M1 = []
T_M2 = []; PWM_M2 = []; ef_M2 = []; Preq_M2 = []
T_M3 = []; PWM_M3 = []; ef_M3 = []; Preq_M3 = []
T_M4 = []; PWM_M4 = []; ef_M4 = []; Preq_M4 = []
T_M5 = []; PWM_M5 = []; ef_M5 = []; Preq_M5 = []
T_M6 = []; PWM_M6 = []; ef_M6 = []; Preq_M6 = []
T_M7 = []; PWM_M7 = []; ef_M7 = []; Preq_M7 = []
T_M8 = []; PWM_M8 = []; ef_M8 = []; Preq_M8 = []
OP = []
STATUS = []
v = []
vz = []
w = []
dist_percorr = [0]
dist_pulv = [0]
Preq_prop = []; P_gerador = [] 
t_voo = []
X_rtl_por_voo = []
Y_rtl_por_voo = []
Massa_por_voo = []
M_retorno = []
Ebat_retorno = []
Tempo_por_voo = []
Delta_x = [0]
Delta_y = [0]
cons_pulv = []
M_tot = []
Massa_por_voo.append(M_pulv_max)
preco_drone_bateria = np.zeros(len(volume_tanque))
preco_por_ha_real = np.zeros(len(volume_tanque))

while M_pulv_max <= M_pulv_lim:
    M_tot_in = M_pulv_max + M_vazio + M_bat
    
    P_sensores = 0;
    P_LED = 100
    P_sist = 38.71
    P_bombas = 95.04
    dt = 0.1
    t_prep = 0; t_abs_calda = 0; t_abs_comb = 0; t_desloc_pre_op = 4.5 
    t_desloc_pos_op = 0 ;
    t_triplice_lavagem = 0;
    t_lavagem_limpeza = 0
    t = []; t.append(0)
    j = 0
    flag = "off"
    
    while True:
        n_abs = 1
        n = 1
        voo = 1     
        iii = 0
        M_pulv = []; M_pulv.append(massa_joao[iii])

        t = []; t.append(t_prep + t_desloc_pre_op)
        t_pulv = []; t_pulv.append(0.0)
        t_manobra = []; t_manobra.append(0.0)
        t_de_voo = []; t_de_voo.append(0)

        X = X0 + j
        Z = zi
        theta_dir = 0
        xi = x0 + faixa/2

        x = []; x_rtl = 0
        y = []; y_rtl = 0
        z = []; z_rtl = 0
        theta = []; theta_rtl = 0
        alpha = 0
        
        x.append(0.0)
        y.append(0.0)
        z.append(0.0)
        theta.append(0.0)
        thetai = math.atan(xi/yi)*180/math.pi
        autonomia = [];
        dist_rtl = [];
        produtiv_por_voo = []
        
        M_tot.append(M_tot_in)

        P_bat = []
        Preq_tot = [];
        E_bat = []; E_bat.append(E_bat_max)
        
        i = 0
        
        OP.append("DESLOCANDO")
        
#OPERANDO ================================================================================================#  

        while OP[i] != "FIM":
            
            if voo == 1:
                z_deslocando = 14
            elif voo == 2:
                z_deslocando = 18
            elif voo == 3:
                z_deslocando = 19
            elif voo == 4:
                z_deslocando = 25
            elif voo == 5:
                z_deslocando = 27
            elif voo == 6:
                z_deslocando = 30
            elif voo == 7:
                z_deslocando = 37
            elif voo == 8:
                z_deslocando = 41
#DESLOCANDO ==============================================================================================# 
            
            if voo == 1 and OP[i] == "DESLOCANDO":
                v_desloc = v_pulv-0.0001
            else:
                v_desloc = v_deslocamento
            
            if OP[i] == "DESLOCANDO":
                zi = z_deslocando
                if z[i] < zi:
                    vz.append(v_subida)
                    v.append(0.0)
                    w.append(0.0)
                    STATUS.append("SUBIDA")
                elif theta[i] < thetai:
                    vz.append(0.0)
                    v.append(0.0)
                    w.append(omega)
                    STATUS.append("YAW+")
                elif theta[i] > thetai:
                    vz.append(0.0)
                    v.append(0.0)
                    w.append(-omega)
                    STATUS.append("YAW-")
                elif ((x[i]**2 + y[i]**2)**(1/2) < (xi**2 + yi**2)**(1/2) - 1*(v_desloc**2)/(2*acel)) and v[i-1] < v_desloc:
                    vz.append(0.0)
                    v.append(v[i-1]+dt*acel)
                    w.append(0.0)
                    STATUS.append("PITCH acelerando")
                elif ((x[i]**2 + y[i]**2)**(1/2) < (xi**2 + yi**2)**(1/2) - 1*(v_desloc**2)/(2*acel)) and v[i-1] >= v_desloc:
                    vz.append(0.0)
                    v.append(v_desloc)
                    w.append(0.0)
                    STATUS.append("PITCH")
                elif ((x[i]**2 + y[i]**2)**(1/2) >= (xi**2 + yi**2)**(1/2) - 1*(v_desloc**2)/(2*acel)) and v[i-1] > 0:
                    vz.append(0.0)
                    v.append(v[i-1]-dt*acel)
                    w.append(0.0)
                    STATUS.append("PITCH desacelerando")
                elif ((x[i]**2 + y[i]**2)**(1/2) >= (xi**2 + yi**2)**(1/2) - 1*(v_desloc**2)/(2*acel)) and v[i-1] <= 0:
                    vz.append(0.0)
                    v.append(0)
                    w.append(0.0)
                    STATUS.append("PITCH")
                    x[i] = xi
                    y[i] = yi

                if STATUS[i] == "YAW+":
                    if (theta[i] + w[i] * dt > thetai or theta[i] + w[i] * dt == thetai ):
                        theta.append(thetai)
                    else:
                        theta.append(theta[i] + w[i] * dt)
                elif STATUS[i] == "YAW-":
                    if (theta[i] + w[i] * dt < thetai or theta[i] + w[i] * dt == thetai ):
                        theta.append(thetai)
                    else:
                        theta.append(theta[i] + w[i] * dt)
                else:
                    theta.append(theta[i])
                    
                if (z[i] + vz[i] * dt > zi):
                    z.append(zi)
                else:
                    z.append(z[i] + vz[i] * dt)
                    
                if (x[i] + v[i] * math.sin(math.radians(theta[i])) * dt > xi):
                    x.append(xi)
                else:
                    x.append(x[i] + v[i] * math.sin(math.radians(theta[i])) * dt)
                    
                if ((y[i] + v[i] * math.cos(math.radians(theta[i])) * dt) > yi):
                    y.append(yi)
                else:
                    y.append(y[i] + v[i] * math.cos(math.radians(theta[i])) * dt)
                    
#PULVERIZANDO ==============================================================================================#    
        
            if OP[i] == "PULVERIZANDO":
                if z[i] >= z_pulverizando:
                    vz.append(-v_subida)
                    v.append(0.0)
                    w.append(0.0)
                    STATUS.append("DESCIDA")
                elif ((theta[i] == theta_dir and y[i] < (yi + Y - (v_pulv**2 - v_yaw**2)/(2*acel)) and v[i-1] < v_pulv) or (theta[i] == theta_dir + 180 and y[i] > yi + (v_pulv**2 - v_yaw**2)/(2*acel)  and v[i-1] < v_pulv)):
                    vz.append(0.0)
                    v.append(v[i-1]+dt*acel)
                    w.append(0.0)
                    STATUS.append("PITCH acelerando")
                elif ((theta[i] == theta_dir and y[i] < (yi + Y - (v_pulv**2 - v_yaw**2)/(2*acel)) and v[i-1] >= v_pulv) or (theta[i] == theta_dir + 180 and y[i] > yi + (v_pulv**2 - v_yaw**2)/(2*acel) and v[i-1] >= v_pulv)):
                    vz.append(0.0)
                    v.append(v_pulv)
                    w.append(0.0)
                    STATUS.append("PITCH")
                elif (theta[i] >= theta_dir and y[i] >= (yi + Y - (v_pulv**2 - v_yaw**2)/(2*acel)) and v[i-1] > v_yaw):
                    vz.append(0.0)
                    w.append(0.0)
                    v.append(v[i-1]-dt*acel)
                    STATUS.append("PITCH desacelerando")
                elif (theta[i] >= theta_dir and y[i] >= (yi + Y - (v_pulv**2 - v_yaw**2)/(2*acel)) and v[i-1] <= v_yaw):
                    vz.append(0.0)
                    w.append(omega)
                    v.append(v_yaw)
                    STATUS.append("YAW+")
                    if STATUS[i-1] == "PITCH desacelerando":
                        n_passada = n_passada + 1
                        print(x[i], n_passada)
                elif (theta[i] <= theta_dir + 180 and y[i] <= (yi + (v_pulv**2 - v_yaw**2)/(2*acel)) and v[i-1] > v_yaw):
                    vz.append(0.0)
                    w.append(0.0)
                    v.append(v[i-1]-dt*acel)
                    STATUS.append("PITCH desacelerando")
                elif (theta[i] <= theta_dir + 180 and y[i] <= (yi + (v_pulv**2 - v_yaw**2)/(2*acel)) and v[i-1] <= v_yaw):
                    vz.append(0.0)
                    w.append(-omega)
                    v.append(v_yaw)
                    STATUS.append("YAW-")
                    if STATUS[i-1] == "PITCH desacelerando":
                        n_passada = n_passada + 1
                        print(x[i], n_passada)
                    
                x.append(x[i] + v[i] * math.sin(math.radians(theta[i])) * dt)
                y.append(y[i] + v[i] * math.cos(math.radians(theta[i])) * dt)
                z.append(z[i] + vz[i] * dt)
                
                if STATUS[i] == "YAW+":
                    if (theta[i] + w[i] * dt >= theta_dir + 180):
                        theta.append(theta_dir + 180)
                        x[i+1] = xi + n * faixa * math.cos(math.radians(theta_dir))
                        y[i+1] = yi + Y
                        n = n + 1
                    else:
                        theta.append(theta[i] + w[i] * dt)
                elif STATUS[i] == "YAW-":
                    if (theta[i] + w[i] * dt <= theta_dir):
                        theta.append(theta_dir)
                        x[i+1] = xi + n * faixa * math.cos(math.radians(theta_dir))
                        y[i+1] = yi
                        n = n + 1
                    else:
                        theta.append(theta[i] + w[i] * dt)
                else:
                    theta.append(theta[i])  
            
#RTL =========================================================================================================# 
        
            if OP[i] == "RTL CALDA" or OP[i] == "RTL BAT" or OP[i] == "RTL FIM":
                if z[i] < z_deslocando and x[i] != 0 and v[i-1] > 0:
                    vz.append(0)
                    v.append((v[i-1]-dt*acel))
                    w.append(0)
                    STATUS.append("PITCH desacelerando")
                elif z[i] < z_deslocando and x[i] != 0 and v[i-1] <= 0:
                    vz.append(v_subida)
                    v.append(0)
                    w.append(0)
                    STATUS.append("SUBIDA")
                elif theta_rtl == theta_dir and theta[i] > -alpha:
                    vz.append(0.0)
                    v.append(0.0)
                    w.append(-omega)
                    STATUS.append("YAW-")
                elif theta_rtl == theta_dir + 180 and theta[i] < alpha + 180:
                    vz.append(0.0)
                    v.append(0.0)
                    w.append(omega)
                    STATUS.append("YAW+")
                elif ((x[i]**2 + y[i]**2)**(1/2) > 1*(v_desloc**2)/(2*acel)) and v[i-1] < v_desloc:
                    vz.append(0.0)
                    v.append((v[i-1]+dt*acel))
                    w.append(0.0)
                    STATUS.append("PITCH acelerando")
                elif ((x[i]**2 + y[i]**2)**(1/2) > 1*(v_desloc**2)/(2*acel)) and v[i-1] >= v_desloc:
                    vz.append(0.0)
                    v.append(v_desloc)
                    w.append(0.0)
                    STATUS.append("PITCH")
                elif ((x[i]**2 + y[i]**2)**(1/2) <= 1*(v_desloc**2)/(2*acel)) and v[i-1] > 0:
                    vz.append(0.0)
                    v.append(v[i-1]-dt*acel)
                    w.append(0.0)
                    STATUS.append("PITCH desacelerando")
                    if x[i] < 0.5 or y[i] < 0.5:
                        x[i] = 0
                        y[i] = 0
                        v[i] = 0
                elif z[i] > 0:
                    vz.append(-v_subida)
                    v.append(0.0)
                    w.append(0.0)
                    STATUS.append("DESCIDA")

                if STATUS[i] == "YAW+":
                    if (theta[i] + w[i] * dt > alpha + 180):
                        theta.append(alpha + 180)
                    else:
                        theta.append(theta[i] + w[i] * dt)
                elif STATUS[i] == "YAW-":
                    if (theta[i] + w[i] * dt < -alpha):
                        theta.append(-alpha)
                    else:
                        theta.append(theta[i] + w[i] * dt)
                else:
                    theta.append(theta[i])
                    
                if (z[i] + vz[i] * dt < 0):
                    z.append(0)
                else:
                    z.append(z[i] + vz[i] * dt)
                    
                if STATUS[i] == "PITCH desacelerando" and abs(z[i] - z_pulverizando) <= 0.001:
                    
                    if (x[i] - abs(v[i]*math.sin(math.radians(theta[i])) * dt) < 0):
                        x.append(0)
                    else:
                        x.append(x[i] - abs(v[i]*math.sin(math.radians(theta[i])) * dt))
                        
                    if (y[i] - abs(v[i] * math.cos(math.radians(theta[i])) * dt) < 0):
                        y.append(0)
                    else:
                        y.append(y[i] + (v[i] * math.cos(math.radians(theta[i])) * dt))
                    
                else:
                    
                    if (x[i] - abs(v[i]*math.sin(math.radians(theta[i])) * dt) < 0):
                        x.append(0)
                    else:
                        x.append(x[i] - abs(v[i]*math.sin(math.radians(theta[i])) * dt))
                        
                    if (y[i] - abs(v[i] * math.cos(math.radians(theta[i])) * dt) < 0):
                        y.append(0)
                    else:
                        y.append(y[i] - abs(v[i] * math.cos(math.radians(theta[i])) * dt))
    
#RTW =========================================================================================================#   

            if OP[i] == "RTW":
                if theta_rtl == theta_dir:
                    if z[i] < z_rtl:
                        vz.append(v_subida)
                        v.append(0.0)
                        w.append(0.0)
                        STATUS.append("SUBIDA")
                    elif theta[i] > alpha and x[i] == 0:
                        vz.append(0.0)
                        v.append(0.0)
                        w.append(-omega)
                        STATUS.append("YAW-")
                    elif ((x[i]**2 + y[i]**2)**(1/2) < (x_rtl**2 + y_rtl**2)**(1/2) - 1*(v_desloc**2)/(2*acel)) and v[i-1] < v_desloc:
                        vz.append(0.0)
                        v.append((v[i-1]+dt*acel))
                        w.append(0.0)
                        STATUS.append("PITCH acelerando")
                    elif ((x[i]**2 + y[i]**2)**(1/2) < (x_rtl**2 + y_rtl**2)**(1/2) - 1*(v_desloc**2)/(2*acel)) and v[i-1] >= v_desloc:
                        vz.append(0.0)
                        v.append(v_desloc)
                        w.append(0.0)
                        STATUS.append("PITCH")
                    elif ((x[i]**2 + y[i]**2)**(1/2) >= (x_rtl**2 + y_rtl**2)**(1/2) - 1*(v_desloc**2)/(2*acel)) and v[i-1] > 0:
                        vz.append(0.0)
                        v.append(v[i-1]-dt*acel)
                        w.append(0.0)
                        STATUS.append("PITCH desacelerando")
                        if abs(x[i]-x_rtl) < 0.5 or abs(y[i]-y_rtl) < 0.5:
                            x[i] = x_rtl
                            y[i] = y_rtl
                            v[i] = 0
                    elif theta[i] > theta_rtl and x[i] == x_rtl:
                          vz.append(0.0)
                          v.append(0.0)
                          w.append(-omega)
                          STATUS.append("YAW-2")
                         
                    if STATUS[i] == "YAW-" and theta[i] + w[i] * dt < alpha:
                        theta.append(alpha)
                    elif STATUS[i] == "YAW-2" and theta[i] + w[i] * dt < theta_rtl:
                        theta.append(theta_rtl)
                    else:
                        theta.append(theta[i] + w[i] * dt)
                        
                    if (z[i] + vz[i] * dt > z_rtl):
                          z.append(z_rtl)
                    else:
                          z.append(z[i] + vz[i] * dt)
                         
                    if (x[i] + v[i] * math.sin(math.radians(theta[i])) * dt > x_rtl):
                          x.append(x_rtl)
                    else:
                          x.append(x[i] + v[i] * math.sin(math.radians(theta[i])) * dt)
                         
                    if ((y[i] + v[i] * math.cos(math.radians(theta[i])) * dt) > y_rtl):
                          y.append(y_rtl)
                    else:
                          y.append(y[i] + v[i] * math.cos(math.radians(theta[i])) * dt)
                         
                elif theta_rtl == theta_dir + 180:
                    if z[i] < z_rtl:
                        vz.append(v_subida)
                        v.append(0.0)
                        w.append(0.0)
                        STATUS.append("SUBIDA")
                    elif theta[i] > alpha and x[i] < x_rtl:
                        vz.append(0.0)
                        v.append(0.0)
                        w.append(-omega)
                        STATUS.append("YAW-")
                    elif ((x[i]**2 + y[i]**2)**(1/2) < (x_rtl**2 + y_rtl**2)**(1/2) - 1*(v_desloc**2)/(2*acel)) and v[i-1] < v_desloc:
                        vz.append(0.0)
                        v.append((v[i-1]+dt*acel))
                        w.append(0.0)
                        STATUS.append("PITCH acelerando")
                    elif ((x[i]**2 + y[i]**2)**(1/2) < (x_rtl**2 + y_rtl**2)**(1/2) - 1*(v_desloc**2)/(2*acel)) and v[i-1] >= v_desloc:
                        vz.append(0.0)
                        v.append(v_desloc)
                        w.append(0.0)
                        STATUS.append("PITCH")
                    elif ((x[i]**2 + y[i]**2)**(1/2) >= (x_rtl**2 + y_rtl**2)**(1/2) - 1*(v_desloc**2)/(2*acel)) and v[i-1] > 0:
                        vz.append(0.0)
                        v.append(v[i-1]-dt*acel)
                        w.append(0.0)
                        STATUS.append("PITCH desacelerando")
                        if abs(x[i]-x_rtl) < 0.5 or abs(y[i]-y_rtl) < 0.5:
                            x[i] = x_rtl
                            y[i] = y_rtl
                            v[i] = 0
                    elif theta[i] < theta_rtl:
                          vz.append(0.0)
                          v.append(0.0)
                          w.append(omega)
                          STATUS.append("YAW+")
                         
                    if STATUS[i] == "YAW-" and theta[i] + w[i] * dt < alpha:
                        theta.append(alpha)
                    elif STATUS[i] == "YAW+" and theta[i] + w[i] * dt > theta_rtl:
                        theta.append(theta_rtl)
                    else:
                        theta.append(theta[i] + w[i] * dt)
                         
                    if (z[i] + vz[i] * dt > z_rtl):
                          z.append(z_rtl)
                    else:
                          z.append(z[i] + vz[i] * dt)
                         
                    if (x[i] + v[i] * math.sin(math.radians(theta[i])) * dt > x_rtl):
                          x.append(x_rtl)
                    else:
                          x.append(x[i] + v[i] * math.sin(math.radians(theta[i])) * dt)
                         
                    if ((y[i] + v[i] * math.cos(math.radians(theta[i])) * dt) > y_rtl):
                          y.append(y_rtl)
                    else:
                          y.append(y[i] + v[i] * math.cos(math.radians(theta[i])) * dt)
            
            
#CONSUMO DE POTÊNCIA ==============================================================================================# 

            T_hover.append(M_tot[i]/n_motores)      # [kg]
            # Cnst_PWM_T = 0.02209*Diametro - 0.43406
            Cnst_PWM_T = 0.3844
            if n_motores == 8:
                
                T_M1.append(T_hover[i] + Cnst_PWM_T * (- 0.8 * v[i] - 0.4035 * w[i] + 3.5 * vz[i]))
                if T_M1[i] < 0.05*T_hover[i]:
                    T_M1[i] = 0.05*T_hover[i]
                ef_M1.append(1000/9.81/(cnst*(np.sqrt(T_M1[i]*9.81/(2*rho*A)))))
                Preq_M1.append(COAXIAL_80*(1000 * T_M1[i]/ef_M1[i]))
                
                T_M2.append(T_hover[i] + Cnst_PWM_T * (0.8 * v[i] - 0.4035 * w[i] + 3.5 * vz[i]))
                if T_M2[i] < 0.05*T_hover[i]:
                    T_M2[i] = 0.05*T_hover[i]
                ef_M2.append(1000/9.81/(cnst*(np.sqrt(T_M2[i]*9.81/(2*rho*A)))))
                Preq_M2.append(COAXIAL_80*(1000 * T_M2[i]/ef_M2[i]))
                
                T_M3.append(T_hover[i] + Cnst_PWM_T * (- 0.23 * v[i] + 0.4035 * w[i] + 3.5 * vz[i]))
                if T_M3[i] < 0.05*T_hover[i]:
                    T_M3[i] = 0.05*T_hover[i]
                ef_M3.append(1000/9.8/(cnst*(np.sqrt(T_M3[i]*9.8/(2*rho*A)))))
                Preq_M3.append(COAXIAL_80*(1000 * T_M3[i]/ef_M3[i]))
                
                T_M4.append(T_hover[i] + Cnst_PWM_T * (0.8 * v[i] + 0.4035 * w[i] + 3.5 * vz[i]))
                if T_M4[i] < 0.05*T_hover[i]:
                    T_M4[i] = 0.05*T_hover[i]
                ef_M4.append(1000/9.81/(cnst*(np.sqrt(T_M4[i]*9.81/(2*rho*A)))))
                Preq_M4.append(COAXIAL_80*(1000 * T_M4[i]/ef_M4[i]))
        
                T_M5.append(T_hover[i] + Cnst_PWM_T * (- 0.8 * v[i] + 0.4035 * w[i] + 3.5 * vz[i]))
                if T_M5[i] < 0.05*T_hover[i]:
                    T_M5[i] = 0.05*T_hover[i]
                ef_M5.append(1000/9.81/(cnst*(np.sqrt(T_M5[i]*9.81/(2*rho*A)))))
                Preq_M5.append(COAXIAL_80*(1000 * T_M5[i]/ef_M5[i]))
                
                T_M6.append(T_hover[i] + Cnst_PWM_T * (0.23 * v[i] + 0.4035 * w[i] + 3.5 * vz[i]))
                if T_M6[i] < 0.05*T_hover[i]:
                    T_M6[i] = 0.05*T_hover[i]
                ef_M6.append(1000/9.8/(cnst*(np.sqrt(T_M6[i]*9.8/(2*rho*A)))))
                Preq_M6.append(COAXIAL_80*(1000 * T_M6[i]/ef_M6[i]))
                
                T_M7.append(T_hover[i] + Cnst_PWM_T * (- 0.23 * v[i] - 0.4035 * w[i] + 3.5 * vz[i]))
                if T_M7[i] < 0.05*T_hover[i]:
                    T_M7[i] = 0.05*T_hover[i]
                ef_M7.append(1000/9.8/(cnst*(np.sqrt(T_M7[i]*9.8/(2*rho*A)))))
                Preq_M7.append(COAXIAL_80*(1000 * T_M7[i]/ef_M7[i]))
                
                T_M8.append(T_hover[i] + Cnst_PWM_T * (0.23 * v[i] - 0.4035 * w[i] + 3.5 * vz[i]))
                if T_M8[i] < 0.05*T_hover[i]:
                    T_M8[i] = 0.05*T_hover[i]
                ef_M8.append(1000/9.8/(cnst*(np.sqrt(T_M8[i]*9.8/(2*rho*A)))))
                Preq_M8.append(COAXIAL_80*(1000 * T_M8[i]/ef_M8[i]))
                
                Preq_prop.append(Preq_M1[i] +  Preq_M2[i] + Preq_M3[i] + Preq_M4[i] + Preq_M5[i] + Preq_M6[i] + Preq_M7[i] + Preq_M8[i])
                
            elif n_motores == 6:
                
                T_M1.append(T_hover[i] + Cnst_PWM_T * (-0.06923890 * v[i] - 0.29355463 * w[i] + 2.70526734 * vz[i]))
                if T_M1[i] < 0.05*T_hover[i]:
                    T_M1[i] = 0.05*T_hover[i]
                ef_M1.append(1000/9.81/(cnst*(np.sqrt(T_M1[i]*9.81/(2*rho*A)))))
                Preq_M1.append(1000 * T_M1[i]/ef_M1[i])
                
                T_M2.append(T_hover[i] +  Cnst_PWM_T * (-0.06923890 * v[i] + 0.29648913 * w[i] + 2.70526734 * vz[i]))
                if T_M2[i] < 0.05*T_hover[i]:
                    T_M2[i] = 0.05*T_hover[i]
                ef_M2.append(1000/9.81/(cnst*(np.sqrt(T_M2[i]*9.81/(2*rho*A)))))
                Preq_M2.append(1000 * T_M2[i]/ef_M2[i])
                
                T_M3.append(T_hover[i] + Cnst_PWM_T * (-0.41846113 * v[i] - 0.29355463 * w[i] + 2.70526734 * vz[i]))
                if T_M3[i] < 0.05*T_hover[i]:
                    T_M3[i] = 0.05*T_hover[i]
                ef_M3.append(1000/9.8/(cnst*(np.sqrt(T_M3[i]*9.8/(2*rho*A)))))
                Preq_M3.append(1000 * T_M3[i]/ef_M3[i])
                
                T_M4.append(T_hover[i] +  Cnst_PWM_T * (0.52299572 * v[i] + 0.29648913 * w[i] + 2.70526734 * vz[i]))
                if T_M4[i] < 0.05*T_hover[i]:
                    T_M4[i] = 0.05*T_hover[i]
                ef_M4.append(1000/9.81/(cnst*(np.sqrt(T_M4[i]*9.81/(2*rho*A)))))
                Preq_M4.append(1000 * T_M4[i]/ef_M4[i])
        
                T_M5.append(T_hover[i] +  Cnst_PWM_T * (-0.41846113 * v[i] + 0.29648913 * w[i] + 2.70526734 * vz[i]))
                if T_M5[i] < 0.05*T_hover[i]:
                    T_M5[i] = 0.05*T_hover[i]
                ef_M5.append(1000/9.81/(cnst*(np.sqrt(T_M5[i]*9.81/(2*rho*A)))))
                Preq_M5.append(1000 * T_M5[i]/ef_M5[i])
                
                T_M6.append(T_hover[i] +  Cnst_PWM_T * (0.52299572 * v[i] - 0.29355463 * w[i] + 2.70526734 * vz[i]))
                if T_M6[i] < 0.05*T_hover[i]:
                    T_M6[i] = 0.05*T_hover[i]
                ef_M6.append(1000/9.8/(cnst*(np.sqrt(T_M6[i]*9.8/(2*rho*A)))))
                Preq_M6.append(1000 * T_M6[i]/ef_M6[i])
                
                Preq_prop.append(Preq_M1[i] +  Preq_M2[i] + Preq_M4[i] + Preq_M5[i] + Preq_M3[i] + Preq_M6[i])
                
            elif n_motores == 4:
                
                T_M1.append(T_hover[i] - 0.1548 * v[i] + 0.0423 * w[i] + 0.8315 * vz[i])
                if T_M1[i] < 0.05*T_hover[i]:
                    T_M1[i] = 0.05*T_hover[i]
                ef_M1.append((1/(cnst * np.sqrt(T_M1[i]*9.81/(2*rho*A) ) )*1000/9.81))
                Preq_M1.append(1000 * T_M1[i]/ef_M1[i])
                
                T_M2.append(T_hover[i] - 0.1548 * v[i] - 0.0423 * w[i] + 0.8315 * vz[i])
                if T_M2[i] < 0.05*T_hover[i]:
                    T_M2[i] = 0.05*T_hover[i]
                ef_M2.append((1/(cnst * np.sqrt(T_M2[i]*9.81/(2*rho*A) ) )*1000/9.81))
                Preq_M2.append(1000 * T_M2[i]/ef_M2[i])
                
                T_M3.append(T_hover[i] + 0.1548 * v[i] + 0.0423 * w[i] + 0.8315 * vz[i])
                if T_M3[i] < 0.05*T_hover[i]:
                    T_M3[i] = 0.05*T_hover[i]
                ef_M3.append((1/(cnst * np.sqrt(T_M3[i]*9.81/(2*rho*A) ) )*1000/9.81))
                Preq_M3.append(1000 * T_M3[i]/ef_M3[i])
                
                T_M4.append(T_hover[i] + 0.1548 * v[i] - 0.0423 * w[i] + 0.8315 * vz[i])
                if T_M4[i] < 0.05*T_hover[i]:
                    T_M4[i] = 0.05*T_hover[i]
                ef_M4.append((1/(cnst * np.sqrt(T_M4[i]*9.81/(2*rho*A) ) )*1000/9.81))
                Preq_M4.append(1000 * T_M4[i]/ef_M4[i])
                
                Preq_prop.append(Preq_M1[i] +  Preq_M2[i] + Preq_M3[i] + Preq_M4[i])
            
            if (OP[i] == "PULVERIZANDO" and (STATUS[i] == "PITCH" or STATUS[i] == "PITCH acelerando" or STATUS[i] == "PITCH desacelerando")):
                Preq_tot.append(Preq_prop[i] + P_LED + P_bombas + P_sist)
                cons_pulv.append(vazao*dt/60)
                t_pulv.append(t_pulv[i] + dt)
                dist_pulv.append(dist_pulv[i] + math.sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2 + (z[i+1]-z[i])**2))
                area_pulv_local.append(v[i]*faixa*dt + area_pulv_local[-1])
            else:
                Preq_tot.append(Preq_prop[i] + P_LED + P_sist)
                cons_pulv.append(0.0)
                t_pulv.append(t_pulv[i])
                dist_pulv.append(dist_pulv[i])
                area_pulv_local.append(area_pulv_local[-1])
                
            E_bat.append(E_bat[i] - Preq_tot[i]*dt/3600)
            P_hover = (n_motores*(COAXIAL_80*(1000 * (M_tot[i]/n_motores)/(1000/9.81/(cnst*(np.sqrt((M_tot[i]/n_motores)*9.81/(2*rho*A))))))))
            autonomia.append(3600*E_bat[i]/(Preq_tot[i]))  #(Preq_tot[i]))
            # print(E_bat[i],(Preq_tot[i]*dt/3600))
            
            if M_pulv[i] - cons_pulv[i] < 0:
                M_pulv.append(0)
            else:
                M_pulv.append(M_pulv[i] - cons_pulv[i])
                
            M_tot.append(M_vazio + M_pulv[i+1] + M_bat)
                
            if(STATUS[i] == "YAW+" or STATUS[i] == "YAW-" or STATUS[i] == "YAW-2"):
                t_manobra.append(t_manobra[i] + dt)
            else:
                t_manobra.append(t_manobra[i])
            
            dist_percorr.append(dist_percorr[i] + math.sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2 + (z[i+1]-z[i])**2))
                
            dist_rtl.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
            
            t.append(t[i] + dt)
            t_de_voo.append(t_de_voo[i] + dt)
    
#OP[I+1] ==============================================================================================# 
            if (OP[i] == "RTL FIM"):
                if (x[i+1] == 0 and y[i+1] == 0 and z[i+1] == 0):
                    OP.append("FIM")
                    produtiv_por_voo.append(dist_pulv[i]*faixa)
                    X_rtl_por_voo.append(x_rtl)
                    Y_rtl_por_voo.append(y_rtl)
                    t_voo.append(t_de_voo[i])
                    Tempo_por_voo.append(t[len(t)-1])
                    n_passada = n_passada + 1
                    t[i+1] = t[i+1] + t_lavagem_limpeza + t_triplice_lavagem + t_desloc_pos_op
                    M_retorno.append(M_pulv[i])
                    Ebat_retorno.append(E_bat[i])
                else:
                    OP.append("RTL FIM")
            elif(x[i+1] >= X + x0 and (STATUS[i] == "PITCH" or STATUS[i] == "PITCH acelerando" or STATUS[i] == "PITCH desacelerando")):
                theta_rtl = theta[i+1]
                alpha = math.atan2(x[i+1],y[i+1])*180/math.pi
                x_rtl = x[i+1]
                y_rtl = y[i+1]
                z_rtl = z_deslocando
                n_passada = 1 + n_passada

                OP.append("RTL FIM")
                
            elif(M_pulv[i+1] == 0 and (STATUS[i] != "YAW+" and STATUS[i] != "YAW-" )):
                M_retorno.append(M_pulv[i])
                Ebat_retorno.append(E_bat[i])
                if (x[i+1] == 0 and y[i+1] == 0 and z[i+1] == 0):
                    iii +=1
                    if OTIMIZAR_TANQUE == "SIM":
                    
                        energia_tanque = 0
                        energia_bateria = (cap_bat/1000)*3.7*celulas
                        energia_ida = 0
                        energia_voo = 0
                        energia_volta = 0
                        M_Pulv=0
                        while energia_voo <= energia_bateria*(1-bateria_limite) and M_pulv_max>=M_Pulv+0.5:
                            
                            M_Pulv = M_Pulv+0.5
                            A_helice = np.pi*(0.5*Diametro*0.0254)**2 # [m^2]
                            vazao = Taxa/10000 * (v_pulv*60*faixa) # [L ou kg/min]
                            Massa_Total = M_bat + M_vazio + M_Pulv # [kg]
                            Massinha = np.linspace(M_bat+M_vazio,Massa_Total,1000)
                            W = COAXIAL_80*Massinha*1000/((1000/9.80665/(((1/(eta_escmotor*eta_helice)))*(np.sqrt(Massinha*9.80665/(2*rho*A_helice*n_motores)))))) # [W]
                            energia_ida = (COAXIAL_80*Massa_Total*1000/((1000/9.80665/(((1/(eta_escmotor*eta_helice)))*(np.sqrt(Massa_Total*9.80665/(2*rho*A_helice*n_motores)))))))*((((x_rtl)**2+(y_rtl)**2)**0.5)/v_desloc)/3600
                            energia_tanque = np.trapz(W, Massinha)/(vazao*60) # [Wh]
                            
                            distancia_p_percorrer = (M_Pulv/vazao)*v_pulv*60
                            distancia_talhao = Y #+ np.pi*faixa/2
                            
                            if theta_rtl == 0:
                                if distancia_p_percorrer > distancia_talhao - y_rtl:
                                    delta_x = (1 + int((distancia_p_percorrer - (distancia_talhao - y_rtl))/distancia_talhao))*faixa
                                    delta_y = ((distancia_p_percorrer - (distancia_talhao - y_rtl))/distancia_talhao - int((distancia_p_percorrer - (distancia_talhao - y_rtl))/distancia_talhao))*distancia_talhao
                                else:
                                    delta_x = 0
                                    delta_y = distancia_p_percorrer
                            elif theta_rtl == 180:
                                if distancia_p_percorrer > y_rtl:
                                    delta_x = (1 + int((distancia_p_percorrer - (y_rtl))/distancia_talhao))*faixa
                                    delta_y = ((distancia_p_percorrer - (y_rtl))/distancia_talhao - int((distancia_p_percorrer - (y_rtl))/distancia_talhao))*distancia_talhao
                                else:
                                    delta_x = 0
                                    delta_y = distancia_p_percorrer 
                            
                            if delta_x/faixa % 2 == 0:
                                delta_y = np.cos(np.radians(theta_rtl))*delta_y
                            else:
                                delta_y = np.cos(np.radians(theta_rtl +180))*delta_y
                            
                            energia_subida = ((Massa_Total*9.80665*zi)/3600) + (zi/v_subida)*((COAXIAL_80*Massa_Total*1000/((1000/9.80665/(((1/(eta_escmotor*eta_helice)))*(np.sqrt(Massa_Total*9.80665/(2*rho*A_helice*n_motores))))))))/3600
                            energia_descida =  - (((M_bat + M_vazio))*9.80665*zi/3600) + (zi/v_subida)*(COAXIAL_80*(M_bat+M_vazio)*1000/((1000/9.80665/(((1/(eta_escmotor*eta_helice)))*(np.sqrt((M_bat+M_vazio)*9.80665/(2*rho*A_helice*n_motores)))))))/3600
                            energia_curva = ((Massa_Total*(v_pulv**2)/2)/3600)
                            numero_curvas =  delta_x/faixa 
                            energia_curvas = numero_curvas*energia_curva
                            
                            # energia_volta = (COAXIAL_80*(M_bat+M_vazio)*1000/((1000/9.80665/(((1/(eta_escmotor*eta_helice)))*(np.sqrt((M_bat+M_vazio)*9.80665/(2*rho*A_helice*n_motores)))))))*((((x_rtl+delta_x)**2+(y_rtl+delta_y)**2)**0.5)/v_desloc)/3600
                            energia_voo = (energia_tanque + energia_ida + energia_volta + energia_subida + energia_descida + energia_curvas)*fator_erro_otimização
                            
                        
                        Delta_x.append(delta_x + x_rtl)
                        Delta_y.append(delta_y + y_rtl)
                        Massa_por_voo.append(M_Pulv)
                        M_pulv[i+1] = M_Pulv
                        
                    elif OTIMIZAR_TANQUE == "NAO":
                        M_Pulv[i+1] = massa_joao[iii]
                    indice_voo.append(i+1)    
                    OP.append("RTW")
                    t_abs = 30
                    M_Pulv = M_pulv_max
                    
                    if voo == 1:
                        produtiv_por_voo.append(dist_pulv[i]*faixa)
                        X_rtl_por_voo.append(x_rtl)
                        Y_rtl_por_voo.append(y_rtl)
                        t_voo.append(t_de_voo[i])
                        Tempo_por_voo.append(t[len(t)-1])
                    else:
                        produtiv_por_voo.append(dist_pulv[i]*faixa)
                        X_rtl_por_voo.append(x_rtl)
                        Y_rtl_por_voo.append(y_rtl)
                        t_voo.append(t_de_voo[i])
                        Tempo_por_voo.append(t[len(t)-1])
                    voo = voo + 1
                         
                    
                    E_bat[i+1] = E_bat_max
                    if theta[i+1] < 0:
                        theta[i+1] = theta[i+1] + 180         
                elif (OP[i] == "PULVERIZANDO" or OP[i] == "DESLOCANDO"):
                    theta_rtl = theta[i+1]
                    alpha = math.atan2(x[i+1],y[i+1])*180/math.pi
                    x_rtl = x[i+1]
                    y_rtl = y[i+1]
                    z_rtl = z_deslocando
                    OP.append("RTL CALDA")
                else:
                    OP.append("RTL CALDA")
                
            elif(OP[i] == "RTL BAT" or (E_bat[i] <= bateria_limite*E_bat[0] and STATUS[i] != "YAW+" and STATUS[i] !="YAW-")):
                if (x[i+1] == 0 and y[i+1] == 0 and z[i+1] == 0):
                    iii +=1
                    indice_voo.append(i+1) 
                    OP.append("RTW")
                    t_abs = 30
                    E_bat[i+1] = E_bat_max
                    M_retorno.append(M_pulv[i])
                    Ebat_retorno.append(E_bat[i])
                    
                    if iii == len(massa_joao):
                        break
                    
                    if OTIMIZAR_TANQUE == "NAO":
                        M_pulv[i+1] = massa_joao[iii]
                        
                    elif OTIMIZAR_TANQUE == "SIM":
                    
                        energia_tanque = 0
                        energia_bateria = (cap_bat/1000)*3.7*celulas
                        energia_ida = 0
                        energia_voo = 0
                        energia_volta = 0
                        M_Pulv = 0
                        while energia_voo <= energia_bateria*(1-bateria_limite) and M_pulv_max >= M_Pulv+0.5:
                            
                            M_Pulv = M_Pulv+0.5
                            A_helice = np.pi*(0.5*Diametro*0.0254)**2 # [m^2]
                            vazao = Taxa/10000 * (v_pulv*60*faixa) # [L ou kg/min]
                            Massa_Total = M_bat + M_vazio + M_Pulv # [kg]
                            Massinha = np.linspace(M_bat+M_vazio,Massa_Total,1000)
                            W = COAXIAL_80*Massinha*1000/((1000/9.80665/(((1/(eta_escmotor*eta_helice)))*(np.sqrt(Massinha*9.80665/(2*rho*A_helice*n_motores)))))) # [W]
                            energia_ida = (COAXIAL_80*Massa_Total*1000/((1000/9.80665/(((1/(eta_escmotor*eta_helice)))*(np.sqrt(Massa_Total*9.80665/(2*rho*A_helice*n_motores)))))))*((((x_rtl)**2+(y_rtl)**2)**0.5)/v_desloc)/3600
                            energia_tanque = np.trapz(W, Massinha)/(vazao*60) # [Wh]
                            
                            distancia_p_percorrer = (M_Pulv/vazao)*v_pulv*60
                            distancia_talhao = Y #+ np.pi*faixa/2
                            
                            if theta_rtl == 0:
                                if distancia_p_percorrer > distancia_talhao - y_rtl:
                                    delta_x = (1 + int((distancia_p_percorrer - (distancia_talhao - y_rtl))/distancia_talhao))*faixa
                                    delta_y = ((distancia_p_percorrer - (distancia_talhao - y_rtl))/distancia_talhao - int((distancia_p_percorrer - (distancia_talhao - y_rtl))/distancia_talhao))*distancia_talhao
                                else:
                                    delta_x = 0
                                    delta_y = distancia_p_percorrer
                            elif theta_rtl == 180:
                                if distancia_p_percorrer > y_rtl:
                                    delta_x = (1 + int((distancia_p_percorrer - (y_rtl))/distancia_talhao))*faixa
                                    delta_y = ((distancia_p_percorrer - (y_rtl))/distancia_talhao - int((distancia_p_percorrer - (y_rtl))/distancia_talhao))*distancia_talhao
                                else:
                                    delta_x = 0
                                    delta_y = distancia_p_percorrer 
                            
                            if delta_x/faixa % 2 == 0:
                                delta_y = np.cos(np.radians(theta_rtl))*delta_y
                            else:
                                delta_y = np.cos(np.radians(theta_rtl +180))*delta_y
                            
                            energia_subida = ((Massa_Total*9.80665*zi)/3600) + (zi/v_subida)*((COAXIAL_80*Massa_Total*1000/((1000/9.80665/(((1/(eta_escmotor*eta_helice)))*(np.sqrt(Massa_Total*9.80665/(2*rho*A_helice*n_motores))))))))/3600
                            energia_descida =  - (((M_bat + M_vazio))*9.80665*zi/3600) + (zi/v_subida)*(COAXIAL_80*(M_bat+M_vazio)*1000/((1000/9.80665/(((1/(eta_escmotor*eta_helice)))*(np.sqrt((M_bat+M_vazio)*9.80665/(2*rho*A_helice*n_motores)))))))/3600
                            energia_curva = ((Massa_Total*(v_pulv**2)/2)/3600)
                            numero_curvas =  delta_x/faixa 
                            energia_curvas = numero_curvas*energia_curva
                            
                            # energia_volta = (COAXIAL_80*(M_bat+M_vazio)*1000/((1000/9.80665/(((1/(eta_escmotor*eta_helice)))*(np.sqrt((M_bat+M_vazio)*9.80665/(2*rho*A_helice*n_motores)))))))*((((x_rtl+delta_x)**2+(y_rtl+delta_y)**2)**0.5)/v_desloc)/3600
                            energia_voo = (energia_tanque + energia_ida + energia_volta + energia_subida + energia_descida + energia_curvas)*fator_erro_otimização
                        
                        M_Pulv = M_Pulv-0.5
                        Massa_por_voo.append(M_Pulv)
                        Delta_x.append(delta_x + x_rtl)
                        Delta_y.append(delta_y + y_rtl)
                        M_pulv[i+1] = M_Pulv
                    
                    #------ TIRAR SE DER RUIM-------------#
                
                    elif OP[i] == "RTW":
                        OP.append("RTL FIM")
                    #--------------------------------------#
                      
                    n_abs = n_abs + 1
                    t[i+1] = t[i+1] + t_abs_calda
                    M_Pulv = M_pulv_max

                    if voo == 1:
                        produtiv_por_voo.append(dist_pulv[i]*faixa)
                        X_rtl_por_voo.append(x_rtl)
                        Y_rtl_por_voo.append(y_rtl)
                        t_voo.append(t_de_voo[i])
                        Tempo_por_voo.append(t[len(t)-1])
                    else:
                        produtiv_por_voo.append(dist_pulv[i]*faixa)
                        X_rtl_por_voo.append(x_rtl)
                        Y_rtl_por_voo.append(y_rtl)
                        t_voo.append(t_de_voo[i])
                        Tempo_por_voo.append(t[len(t)-1])
                    voo = voo + 1
                    

                    if theta[i+1] < 0:
                        theta[i+1] = theta[i+1] + 180   
                elif (OP[i] != "RTL BAT"):
                    theta_rtl = theta[i+1]
                    alpha = math.atan2(x[i+1],y[i+1])*180/math.pi
                    x_rtl = x[i+1]
                    y_rtl = y[i+1]
                    z_rtl = z_deslocando
                    OP.append("RTL BAT") 
                else:
                    OP.append("RTL BAT")

            elif(OP[i] =="DESLOCANDO"):
                if (x[i+1] == xi and y[i+1] ==  yi and z[i+1] == zi and theta[i+1] == thetai):
                    if thetai == theta_dir:
                        OP.append("PULVERIZANDO")
                    else:   
                        thetai = theta_dir
                        OP.append("DESLOCANDO")
                else:
                    OP.append("DESLOCANDO")
            elif(OP[i] == "RTW"):
                if (x[i+1] == x_rtl and y[i+1] ==  y_rtl and theta[i+1] == theta_rtl):
                    OP.append("PULVERIZANDO")
                else:
                    OP.append(OP[i])
            else:
                OP.append(OP[i])
            i = i + 1
       
        # print(M_pulv_max, X, Y, max(t)/3600, flag)
        # if flag == "on" and max(t)/3600 < horas_maximas:
        #     break
        # elif max(t)/3600 < horas_maximas and flag == "off":
        #     j = j + dt*v_pulv
        # elif max(t)/3600 > horas_maximas:
        #     j = j - dt*v_pulv
        #     flag = "on"
        # else:
        #     break
        
        if flag == "on" and X >= np.sqrt(X0):
            break
        else:
            break
           
        # print(M_pulv_max, X, Y, max(t)/3600, flag)
        # if flag == "on" and max(t)/3600 < horas_maximas:
        #     break
        # elif max(t)/3600 < horas_maximas and flag == "off":
        #     j = j + dt*v_pulv
        # elif max(t)/3600 > horas_maximas:
        #     j = j - dt*v_pulv
        #     flag = "on"
        # else:
        #     break

            
            
            
    #---------------- VETORES PARA RESULTADOS ---------------------------------#
    
    faixa_vector.append(faixa)
    tempo_missao.append(max(t)/3600)
    # dias.append(math.ceil(area_total/(X*Y/10**4)))
    MTOW.append(M_tot_in)
    produtividade.append(X*Y/10**4/(max(t)/3600))
    talhao_maximus.append(X*Y/10**4)
    voo_vector.append(voo)
    area_por_voo.append(X*Y/10**4/voo)
    vazao_bombas.append(vazao)
    dist_percorrida.append(dist_percorr[i])
    dist_pulverizando.append(dist_pulv[i])
    EOC_km.append(dist_pulv[i]/dist_percorr[i])
    EOC_hr.append(t_pulv[i]/t[i])
    abastecimentos.append(n_abs)
    capacidade_vector.append(cap_bat)
    M_pulv_max2.append(M_pulv_max)
    
    # (preco_por_ha_real,preco_drone_bateria) = custos(M_pulv_max,cap_bat,area_total,t_de_voo[i]/3600,voo,math.ceil(area_total/(X**2/10**4)),X**2/10**4)
    # capex = produtividade/preco_drone_bateria
    # tempo_util.append(t_de_voo[i]/3600)
    
    # tempo_manobra.append(t_manobra[i]/3600)
    # tempo_por_voo.append(t_de_voo[i]/voo/60)
    
    # print("-->",M_pulv_max,faixa, X, max(t)/3600,capex,M_pulv_max)

    
    # (preco_por_ha_real[tanque],preco_drone_bateria[tanque]) = custos(M_pulv_max,cap_bat,area_total,t_de_voo[i]/3600,voo,math.ceil(area_total/(X**2/10**4)),X**2/10**4)
    # capex = produtividade/preco_drone_bateria[tanque]
    tempo_util.append(t_de_voo[i]/3600)
    
    tempo_manobra.append(t_manobra[i]/3600)
    # vol_conb_consumido.append(comb_cons[i]/0.715)
    tempo_por_voo.append(t_de_voo[i]/voo/60)
    
    X0 = X - 10; Y0 = X0
    print("-->",M_pulv_max,faixa, X, Y, (max(t)/3600-t[0]/3600- t_lavagem_limpeza/3600 - t_triplice_lavagem/3600 - t_desloc_pos_op/3600))
    tanque = tanque + 1
    M_pulv_max = M_pulv_max + delta_pulv
            
#======================= PLOTS =======================================
teste = [0]
data = {
    "Faixa [m]": faixa_vector,
    "Volume tanque [L]": M_pulv_max2,
    "Tempo de missao [h]": tempo_missao,
    "Tempo operacional dias": dias,
    "MTOW [kg]": MTOW,
    "Capacidade operacional [ha/h]": produtividade,
    "Area pulverizada por dia [ha]": talhao_maximus,
    "N° Voos": voo_vector,
    "Hectare por voo [ha/voo]": area_por_voo,
    "Capacidade bateria [mha]": capacidade_vector,
    "Capacidade bateria 2 [mha]": teste,
    "Vazao [L/min]": vazao_bombas,
    "Distancia percorrida [km]": dist_percorrida,
    "Distancia Pulverizando [km]": dist_pulverizando,
    "EOC [km/km]": EOC_km,
    "EOC [h/h]": EOC_hr,
    "Tempo de manobra [h]": tempo_manobra,
    "CAPEX [ha/h/R$]": capex,
    "Preco por ha real": preco_por_ha_real,
    "Preco drone, carreg e bat": preco_drone_bateria,
    "Preco drone, carreg e bat 2 ": teste,
    "Tempo util [h]": tempo_util,
    "Tempo util 2 [h]": teste,
    "Tempo por voo [min]":tempo_por_voo
}
distancia = [0]  # Inicializa a lista de distâncias com o valor inicial 0
Dist_voo = []
for j in range(len(indice_voo)):  # Itera sobre os índices de 'indice_voo'
    distancia = [0]
    for n in range(indice_voo[j]):  # Itera de 0 até o valor armazenado em 'indice_voo[j]'
        if n+1 == (indice_voo[j]):
            Dist_voo.append(distancia[n])
        if n+1 < len(x):  # Garante que 'n+1' esteja dentro do limite de 'x' e 'y'
            dx = x[n+1] - x[n]  # Calcula a diferença em x
            dy = y[n+1] - y[n]  # Calcula a diferença em y
            distancia.append((dx**2 + dy**2)**0.5 + distancia[n])  # Adiciona a distância acumulada
Dist_voo_corrigida = [Dist_voo[0]]
for n in range(len(Dist_voo)-1):        
    Dist_voo_corrigida.append(Dist_voo[n+1]-Dist_voo[n])
ErroX = []
ErroY = []
ErroRTW = []

for n in range(len(X_rtl_por_voo)):
    ErroX.append(100*(abs(X_rtl_por_voo[n]-X_rtw[n])/X_rtl_por_voo[n]))
    ErroY.append(100*(abs(Y_rtl_por_voo[n]-Y_rtw[n])/Y_rtl_por_voo[n]))
    dist_simulado = ((X_rtl_por_voo[n])**2+(Y_rtl_por_voo[n])**2)**(1/2)
    dist_ensaio =  ((X_rtw[n])**2+(Y_rtw[n])**2)**(1/2)
    ErroRTW.append((abs(dist_simulado-dist_ensaio)))

ErroDistVooRel = []
ErroDistVooAbs = []
for n in range(len(Dist_voo_corrigida)):
    ErroDistVooRel.append(100*(abs(Dist_ensaio_voo[n]-Dist_voo_corrigida[n])/Dist_ensaio_voo[n]))
    ErroDistVooAbs.append(((Dist_ensaio_voo[n]-Dist_voo_corrigida[n])))
    
vol = []
for k in range(len(MTOW)):
    vol.append(str(delta_pulv*k + M_pulv_min) + "[L]")
      

vetor_sem_ultimo = ErroDistVooAbs[:-1]

desvio_rel_0 = np.sqrt(np.mean(np.array(vetor_sem_ultimo)**2))

print(f"{desvio_rel_0}")

# POR_VOO_M_pulv = np.array(M_pulv)
# POR_VOO_E_bat = np.array(E_bat)

POR_VOO_Tempo = np.array(Tempo_por_voo)/3600
POR_VOO_X_rtl = np.array(X_rtl_por_voo)
POR_VOO_Y_rtl = np.array(Y_rtl_por_voo)
POR_VOO_D_rtl = np.array(((POR_VOO_Y_rtl)**2 + (POR_VOO_X_rtl)**2)**0.5)
# POR_VOO_Delta_x = np.array(Delta_x)
# POR_VOO_Delta_y = np.array(Delta_y)
POR_VOO_Produtividade = np.array(produtiv_por_voo)/10000
POR_VOO_Massa = np.array(Massa_por_voo)
POR_VOO_M_Retorno = np.array(M_retorno)
POR_VOO_Ebat_Retorno = np.array(Ebat_retorno)
POR_VOO_T_VOO = np.array(t_voo)/60

produtividade_matriz[it] = produtividade
# capex_matriz[it] = capex
faixa = faixa + 1
# df1 = pd.DataFrame(data, index = vol )
# resultados.append(df1)
it = it + 1

intervalos_cores = [
    (0, 'red'),
    (1, 'blue'),
    (2, 'green'),
    (3, 'purple'),
    (4, 'orange'),
    (5, 'yellow'),
    (6, 'brown'),
    (7, 'pink'),
    (8, 'cyan'),
    (9, 'gray'),
    (10, 'red'),
    (11, 'blue'),
    (12, 'green'),
    (13, 'purple'),
    (14, 'orange'),
    (15, 'yellow'),
    (16, 'brown'),
    (17, 'pink'),
    (18, 'black'),
    (19, 'blue'),
    (20, 'green'),
    (21, 'purple'),
    (22, 'orange'),
    (23, 'yellow'),
    (24, 'brown'),
    (25, 'pink'),
    (26, 'cyan'),
    (27, 'gray'),
    (28, 'red'),
    (29, 'blue'),
    (30, 'green'),
    (31, 'purple'),
    (32, 'orange'),
    (33, 'yellow'),
    (34, 'brown'),
    (35, 'pink'),
    (36, 'black'),
    (37, 'red'),
    (38, 'blue'),
    (39, 'green'),
    (40, 'purple'),
    (41, 'orange'),
    (42, 'yellow'),
    (43, 'brown'),
    (44, 'pink'),
    (45, 'cyan'),
    (46, 'gray'),
    (47, 'red'),
    (48, 'blue'),
    (49, 'green'),
    (50, 'purple'),
]

x = np.array(x)
y = np.array(y)
M_pulv = np.array(M_pulv)
E_bat = [(x / E_bat[0]) * 100 for x in E_bat];E_bat = np.array(E_bat)
v.append(0);v = np.array(v)
vz.append(0);vz = np.array(vz)
t = np.array(t)
z = np.array(z)
theta = np.array(theta)
STATUS.append("FIM")
n_passada = n_passada + 1
area_pulv_local = np.array(area_pulv_local)/10000

copia_p_excel = pd.DataFrame({
    't': t,
    'x': x,
    'y': y,
    'Ebat': E_bat,
    'Mpulv': M_pulv,
    'v': v,
    'z': z,
    'vz': -1*vz,
    'yaw': -1*theta,
    'area acumu': area_pulv_local
})

andamento_operacao = pd.DataFrame({
    't': t,
    'x': x,
    'y': y,
    'z': z,
    'Ebat': E_bat,
    'Mpulv': M_pulv,
    'v': v,
    'vz':vz,
    'OP': OP,
    'STATUS': STATUS,
})

for k in range(len(resultados)):
    resultados[k].to_excel("Faixa de " + str(resultados[k].loc[str(delta_pulv * k + M_pulv_min) + "[L]", "Faixa [m]"]) + " metros 1 ciclo por abs.xlsx")
 
t_horas =np.array(t)/3600
# Função para atribuir cor com base no valor em t
def atribuir_cor(valor_t):
    for intervalo, cor in intervalos_cores:
        if valor_t <= intervalo:
            return cor
    return intervalos_cores[-1][1]
 
# Atribuir cores com base no vetor t
cores = [atribuir_cor(valor) for valor in t_horas]
 
fig_trajeto = go.Figure()
# # Adicionar linhas aos eixos x, y e z
 
fig_trajeto.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', line=dict(color=cores, colorscale='Viridis', width=2),))
 
# Configurar layout
fig_trajeto.update_layout(scene=dict(aspectmode="cube"))
fig_trajeto.update_layout(paper_bgcolor="white", showlegend=False, scene=dict(aspectmode='data'))
#plot(fig_trajeto, auto_open=True)
fig_trajeto.write_html("saida.html")
# axs.axis('equal')