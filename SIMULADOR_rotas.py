import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
from opt_tanque import Otimizador_Tanque
import plotly.graph_objects as go
from RotasFun import rotacionar_ponto, ajustar_offset, distancia_origem, angulo_em_relaçao_ao_eixo_x, ordenar_pontos, calcular_reta

# INPUTS PRINCIPAIS
#=============================================================================#  

n_motores = 8 # Número motores
Diametro = 54 # [in]
eta_escmotor = 0.848 # Eficiência ESC e Motor
eta_helice = 0.7719 # Eficiência Hélice
rho = 1.225 # [g/cm3]
Taxa = 10.0 # [L/ha]      
v_pulv = 7 # [m/s] 
v_deslocamento = 10 # [m/s] 
faixa = 11 # [m] 
celulas = 14 # [m/s] 
cap_bat = 30000*0.81 # [m/Ah]*útil                                                                                                     
M_vazio = 38 # [kg] 
M_bat = 12.9 # [kg] 
COAXIAL_80 = 1.397542375147 # Sobressalência de potência do coaxial
Cnst_PWM_T = 0.3844 # Constante de transformação pwm para tração
fator_erro_otimizacao = 1.1 # Fator para adequar a otimização
bateria_limite = 0.29 # [%] de bateria para RTL BAT
z_deslocando = 14 # [m] 
z_pulverizando = 5.001 # [m] 
acel = 1.4 # [m/s2] 
v_subida = 2 # [m/s] 
omega = 26.0 # [rad/s]
x0 = 42 # [m] Posição Inicial
zi = 5.0 # [m] Altura pulverização referência
Y = 300 # [m] Lado do Talhão
area_total = 17.28 # [ha] 
X0 = area_total*10000/Y # [m] Lado do Talhão
g = 9.80665 # gravidade

# INPUTS DO TALHÃO
#=============================================================================# 

OTIMIZAR_TANQUE = "NAO" #SIM ou NAO para otimizar tanque de cada voo
SETAR_TANQUE = "SIM"  #SIM ou NAO para setar o tanque de cada voo
SETAR_POSICAO = "NAO" #SIM ou NAO para setar a posição de cada voo
SETAR_Z_DESLOCAMENTO = "NAO" #SIM ou NAO para setar o Z de deslocamento em voo

# pontos = [[50, 50], [20, 100], [200, 80], [150, 0]]
# pontos = [[50, 50], [100, 200], [250, 150], [300, 0]]
pontos = [[46.5-faixa/2, 43], [46.5-faixa/2, 334], [606+faixa/2, 71], [606+faixa/2, 357]]
# pontos = [[100, 50], [50, 200], [200, 150], [250, 100]]
thet = 0


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
set_tanque = [35.00,29.50,31.50,31.00,30.00,30.00,30.00,26.50] # [L] para x voos 
Z_rtw = [14,18,19,25,27,30,37,41]
Dist_ensaio_voo = [2511, 2662, 2940, 2817, 2933, 3168, 3071, 2904] # [m] para x voos 

# CÁLCULOS INICIAIS
#=============================================================================# 

iteracao_posicao = -1
E_bat_max = (cap_bat/1000)*3.7*celulas
vazao = Taxa/10000 * (v_pulv*60*faixa)
A = np.pi*(0.5*Diametro*0.0254)**2
cnst = 1/(eta_escmotor*eta_helice)
v_yaw = math.pi/180*omega*faixa/2
M_pulv_max = 35
area_pulv_local = [0]
indice_voo = []
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
Preq_prop = [];
t_voo = []
X_rtl_por_voo = []
Y_rtl_por_voo = []
Massa_por_voo = []
voo_cor = [];voo_cor.append(1)
M_retorno = []
Ebat_retorno = []
Tempo_por_voo = []
cons_pulv = []
M_tot = []
Massa_por_voo.append(M_pulv_max)
n_passada2 = 0

## CALCULO DOS PONTOS DO TALHAO
ponto1, ponto2, ponto3, ponto4 = ordenar_pontos(pontos)
ponto1_rotacionado = rotacionar_ponto(ponto1, thet)
ponto2_rotacionado = rotacionar_ponto(ponto2, thet)
ponto3_rotacionado = rotacionar_ponto(ponto3, thet)
ponto4_rotacionado = rotacionar_ponto(ponto4, thet)
pontos_rotacionados = [ponto1_rotacionado, ponto2_rotacionado, ponto3_rotacionado, ponto4_rotacionado]
pontos_ajustados, offset_x, offset_y = ajustar_offset(pontos_rotacionados)
# ponto1, ponto2, ponto3, ponto4 = pontos_ajustados 
ponto1, ponto2, ponto3, ponto4 = ordenar_pontos(pontos_ajustados)  
coef12 = calcular_reta(ponto1, ponto2)
coef23 = calcular_reta(ponto2, ponto3)
coef34 = calcular_reta(ponto3, ponto4)
coef14 = calcular_reta(ponto1, ponto4)
max_x = max(p[0] for p in [ponto1, ponto2, ponto3, ponto4])
thet = 0
faixa1 = faixa
x1 = []
x11 = []
y1 = []
y_min = []
x2 = []
x22 = []
y2 = []
y_max = []
n_passada = 1
i = 0
while True:
    # if ponto1[0] != ponto2[0]:
    if n_passada == 1:
        faixa = faixa1/2
    else :
        faixa = faixa1
    if coef12[0] >= 0:
        
        if n_passada == 1:
            x1.append(n_passada * faixa + ponto1[0])
            x11.append(n_passada * faixa + ponto1[0] + faixa)
        else:
            x1.append(n_passada * faixa + ponto1[0] - faixa/2)
            x11.append(n_passada * faixa + ponto1[0])
            
        if x1[i] <= ponto4[0]:
            y1.append(coef14[0] * x1[i] + coef14[1])
            
            if coef14[0] >= 0:
                y_min.append(coef14[0] * (x11[i-1]) + coef14[1])
            else:
                if i % 2 == 0:
                    y_min.append(coef14[0] * x1[i] + coef14[1])
                else:
                    y_min.append(coef14[0] * (x11[i]+faixa) + coef14[1])
                
        else:
            y1.append(coef34[0] * x1[i] + coef34[1])
            if coef34[0] >= 0:
                y_min.append(coef34[0] * x1[i] + coef34[1])
            else:
                y_min.append(coef34[0] * (x11[i]+faixa1) + coef34[1])
    
    
        x2.append(x1[i])
        x22.append(x11[i])
        
        if x2[i] <= ponto2[0]:
            y2.append(coef12[0] * x2[i] + coef12[1])
            if i % 2 == 0:
                y_max.append(coef12[0] * (x22[i]+faixa1/2) + coef12[1])
            else:
                y_max.append(coef12[0] * x2[i] + coef12[1])
            if y_max[i]> ponto2[1]:
                y_max[i] = ponto2[1]
           
                
        elif x2[i] > ponto3[0]:
            y2.append(coef34[0] * x2[i] + coef34[1])
            if coef34[0] >= 0:
                y_max.append(coef34[0] * x22[i] + coef34[1])
            elif coef34[0] < 0:
                y_max.append(coef34[0] * (x2[i]) + coef34[1])
                
        else:
            y2.append(coef23[0] * x2[i] + coef23[1])
            if coef23[0] >= 0:
                y_max.append(coef23[0] * x22[i] + coef23[1])
            elif coef23[0] < 0:
                y_max.append(y2[i])
            
    elif coef12[0] < 0:
        
        if n_passada == 1:
            x1.append(n_passada * faixa + ponto2[0])
            x11.append(n_passada * faixa + ponto2[0] + faixa1/2)
        else:
            x1.append(n_passada * faixa + ponto2[0] - faixa/2)
            x11.append(n_passada * faixa + ponto2[0])
            
        if x1[i] <= ponto1[0]:
            
            y1.append(coef12[0] * x1[i] + coef12[1])
            
            if i % 2 == 0:
                y_min.append(coef12[0] * x1[i] + coef12[1])
            else:
                y_min.append(coef12[0] * (x11[i]+faixa1/2) + coef12[1])
                
        elif x1[i] >= ponto4[0]:
            y1.append(coef34[0] * x1[i] + coef34[1])
            if coef34[0] >= 0:
                y_min.append(coef34[0] * (x1[i]) + coef34[1])
            else:
                if i % 2 == 0:
                    y_min.append(coef34[0] * x1[i] + coef34[1])
                else:
                    y_min.append(coef34[0] * (x11[i]-faixa1/2) + coef34[1])
                
        else:
            y1.append(coef14[0] * x1[i] + coef14[1])
            if coef14[0] >= 0:
                y_min.append(coef14[0] * x11[i] + coef14[1])
            else:
                if i % 2 == 0:
                    y_min.append(coef14[0] * x1[i] + coef14[1])
                    
                else:
                    y_min.append(coef14[0] * (x11[i]+faixa1/2) + coef14[1])
                
        x2.append(x1[i])
        x22.append(x1[i])
        if ponto3[0] == max_x:
            y2.append(coef23[0] * x2[i] + coef23[1])
            y_max.append(coef23[0] * x22[i] + coef23[1])
        else:
            if x2[i] <= ponto3[0]:
                y2.append(coef23[0] * x2[i] + coef23[1])
                y_max.append(coef23[0] * x22[i] + coef23[1])
            else:
                y2.append(coef34[0] * x2[i] + coef34[1])
                y_max.append(coef34[0] * x22[i] + coef34[1])
    if x1[i] >= (max_x-faixa/2):
        break
    n_passada += 1
    i += 1

while True:
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
    n_abs = 1
    n = 1
    voo = 1     
    iii = 0
    M_pulv = []; 
    X = max_x
    if SETAR_TANQUE == "SIM":
        M_pulv.append(set_tanque[iii])
    else:
        M_pulv.append(M_pulv_max)
        
    t = []; t.append(t_prep + t_desloc_pre_op)
    t_pulv = []; t_pulv.append(0.0)
    t_manobra = []; t_manobra.append(0.0)
    t_de_voo = []; t_de_voo.append(0)
    Z = zi
    theta_dir = 0
    xi = x1[0]
    yi = y_min[0]
    x = []; x_rtl = 0
    y = []; y_rtl = 0
    z = []; z_rtl = 0
    theta = []; theta_rtl = 0
    alpha_ida = 0
    alpha_volta = 0
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
    
# COMEÇANDO A OPERAÇÃO
#=============================================================================# 

    OP.append("DESLOCANDO")
    while OP[i] != "FIM":
        T_hover.append(M_tot[i]/n_motores) 
        if n_passada2 >= n_passada:
            Y = y_max[-1]
            yi = y_min[-1]
        else:
            Y = y_max[n_passada2]
            yi = y_min[n_passada2]
# SETANDO ALTURA
#=============================================================================# 
        
        if SETAR_Z_DESLOCAMENTO == "SIM":
            for pos in range(len(Z_rtw)):
                if voo == (pos+1):
                    z_deslocando = Z_rtw[pos]
       
# DESLOCANDO
#=============================================================================#  
        
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

# PULVERIZANDO
#=============================================================================#     
        
        if OP[i] == "PULVERIZANDO":
            
            if z[i] >= z_pulverizando:
                vz.append(-v_subida)
                v.append(0.0)
                w.append(0.0)
                STATUS.append("DESCIDA")
            elif ((theta[i] == theta_dir and y[i] < (Y - (v[i-1]**2 - v_yaw**2)/(2*acel)) and v[i-1] < v_pulv) or (theta[i] == theta_dir + 180 and y[i] > yi + (v[i-1]**2 - v_yaw**2)/(2*acel)  and v[i-1] < v_pulv)):
                vz.append(0.0)
                v.append(v[i-1]+dt*acel)
                w.append(0.0)
                STATUS.append("PITCH acelerando")
            elif ((theta[i] == theta_dir and y[i] < (Y - (v_pulv**2 - v_yaw**2)/(2*acel)) and v[i-1] >= v_pulv) or (theta[i] == theta_dir + 180 and y[i] > yi + (v_pulv**2 - v_yaw**2)/(2*acel) and v[i-1] >= v_pulv)):
                vz.append(0.0)
                v.append(v_pulv)
                w.append(0.0)
                STATUS.append("PITCH")
            elif (theta[i] >= theta_dir and y[i] >= (Y - (v[i-1]**2 - v_yaw**2)/(2*acel)) and v[i-1] > v_yaw):
                vz.append(0.0)
                w.append(0.0)
                v.append(v[i-1]-dt*acel)
                STATUS.append("PITCH desacelerando")
            elif (theta[i] >= theta_dir and y[i] >= (Y - (v[i-1]**2 - v_yaw**2)/(2*acel)) and v[i-1] <= v_yaw):
                vz.append(0.0)
                w.append(omega)
                v.append(v_yaw)
                STATUS.append("YAW+")
                if (STATUS[i-1] == "PITCH desacelerando" or STATUS[i-1] == "PITCH acelerando" or STATUS[i-1] == "DESCIDA"):
                    n_passada2 = n_passada2 + 1
            elif (theta[i] <= theta_dir + 180 and y[i] <= (yi + (v[i-1]**2 - v_yaw**2)/(2*acel)) and v[i-1] > v_yaw):
                vz.append(0.0)
                w.append(0.0)
                v.append(v[i-1]-dt*acel)
                STATUS.append("PITCH desacelerando")
            elif (theta[i] <= theta_dir + 180 and y[i] <= (yi + (v_pulv**2 - v_yaw**2)/(2*acel)) and v[i-1] <= v_yaw):
                vz.append(0.0)
                w.append(-omega)
                v.append(v_yaw)
                STATUS.append("YAW-")
                if (STATUS[i-1] == "PITCH desacelerando" or STATUS[i-1] == "PITCH acelerando" or STATUS[i-1] == "DESCIDA"):
                    n_passada2 = n_passada2 + 1
                
            x.append(x[i] + v[i] * math.sin(math.radians(theta[i])) * dt)
            y.append(y[i] + v[i] * math.cos(math.radians(theta[i])) * dt)
            z.append(z[i] + vz[i] * dt)
            
            if STATUS[i] == "YAW+":
                if (theta[i] + w[i] * dt >= theta_dir + 180):
                    theta.append(theta_dir + 180)
                    if x[i] >= max(x1):
                        x[i+1] = max(x1)
                        y[i+1] = Y
                    else:
                        x[i+1] = xi + n * faixa * math.cos(math.radians(theta_dir))
                        y[i+1] = Y
                    n = n + 1
                else:
                    theta.append(theta[i] + w[i] * dt)
            elif STATUS[i] == "YAW-":
                if (theta[i] + w[i] * dt <= theta_dir):
                    theta.append(theta_dir)
                    if x[i] >= max(x1):
                        x[i+1] = max(x1)
                        y[i+1] = yi
                    else:
                        x[i+1] = xi + n * faixa * math.cos(math.radians(theta_dir))
                        y[i+1] = yi
                    n = n + 1
                else:
                    theta.append(theta[i] + w[i] * dt)
            else:
                theta.append(theta[i])  

# RTL
#=============================================================================#  
    
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
            elif theta_rtl == theta_dir and theta[i] > -alpha_volta:
                vz.append(0.0)
                v.append(0.0)
                w.append(-omega)
                STATUS.append("YAW-")
            elif theta_rtl == theta_dir + 180 and theta[i] < alpha_volta + 180:
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
                if (theta[i] + w[i] * dt > alpha_volta + 180):
                    theta.append(alpha_volta + 180)
                else:
                    theta.append(theta[i] + w[i] * dt)
            elif STATUS[i] == "YAW-":
                if (theta[i] + w[i] * dt < -alpha_volta):
                    theta.append(-alpha_volta)
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

# RTW
#=============================================================================#    

        if OP[i] == "RTW":
            if SETAR_POSICAO == "SIM":
                if perna_rtw[iteracao_posicao]%2 == 0:
                    THETA_rtw = 180
                else:
                    THETA_rtw = 0
                theta_rtl = THETA_rtw
                x_rtl = X_rtw[iteracao_posicao]
                y_rtl = Y_rtw[iteracao_posicao]
                alpha_ida = math.atan2(x_rtl,y_rtl)*180/math.pi
                n = perna_rtw[iteracao_posicao]
            
            if z[i] < z_rtl:
                vz.append(v_subida)
                v.append(0.0)
                w.append(0.0)
                STATUS.append("SUBIDA")
            elif theta_rtl == theta_dir and theta[i] > alpha_ida and x[i] == 0:
                vz.append(0.0)
                v.append(0.0)
                w.append(-omega)
                STATUS.append("YAW-")
            elif (theta_rtl == theta_dir + 180) and theta[i] > alpha_ida and x[i] < x_rtl:
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
            elif theta_rtl == theta_dir and theta[i] > theta_rtl and x[i] == x_rtl:
                  vz.append(0.0)
                  v.append(0.0)
                  w.append(-omega)
                  STATUS.append("YAW-2")
            elif (theta_rtl == theta_dir + 180) and theta[i] < theta_rtl:
                  vz.append(0.0)
                  v.append(0.0)
                  w.append(omega)
                  STATUS.append("YAW+")
                     
            if STATUS[i] == "YAW-" and theta[i] + w[i] * dt < alpha_ida:
                theta.append(alpha_ida)
            elif theta_rtl == theta_dir and STATUS[i] == "YAW-2" and theta[i] + w[i] * dt < theta_rtl:
                theta.append(theta_rtl)
            elif theta_rtl == theta_dir + 180 and STATUS[i] == "YAW+" and theta[i] + w[i] * dt > theta_rtl:
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
        
# CÁLCULO DE POTÊNCIA
#=============================================================================#  
   
        if n_motores == 8:
            
            T_M1.append(T_hover[i] + Cnst_PWM_T * (- 0.8 * v[i] - 0.4035 * w[i] + 3.5 * vz[i]))
            if T_M1[i] < 0.05*T_hover[i]:
                T_M1[i] = 0.05*T_hover[i]
            ef_M1.append(1000/g/(cnst*(np.sqrt(T_M1[i]*g/(2*rho*A)))))
            Preq_M1.append(COAXIAL_80*(1000 * T_M1[i]/ef_M1[i]))
            
            T_M2.append(T_hover[i] + Cnst_PWM_T * (0.8 * v[i] - 0.4035 * w[i] + 3.5 * vz[i]))
            if T_M2[i] < 0.05*T_hover[i]:
                T_M2[i] = 0.05*T_hover[i]
            ef_M2.append(1000/g/(cnst*(np.sqrt(T_M2[i]*g/(2*rho*A)))))
            Preq_M2.append(COAXIAL_80*(1000 * T_M2[i]/ef_M2[i]))
            
            T_M3.append(T_hover[i] + Cnst_PWM_T * (- 0.23 * v[i] + 0.4035 * w[i] + 3.5 * vz[i]))
            if T_M3[i] < 0.05*T_hover[i]:
                T_M3[i] = 0.05*T_hover[i]
            ef_M3.append(1000/g/(cnst*(np.sqrt(T_M3[i]*g/(2*rho*A)))))
            Preq_M3.append(COAXIAL_80*(1000 * T_M3[i]/ef_M3[i]))
            
            T_M4.append(T_hover[i] + Cnst_PWM_T * (0.8 * v[i] + 0.4035 * w[i] + 3.5 * vz[i]))
            if T_M4[i] < 0.05*T_hover[i]:
                T_M4[i] = 0.05*T_hover[i]
            ef_M4.append(1000/g/(cnst*(np.sqrt(T_M4[i]*g/(2*rho*A)))))
            Preq_M4.append(COAXIAL_80*(1000 * T_M4[i]/ef_M4[i]))
    
            T_M5.append(T_hover[i] + Cnst_PWM_T * (- 0.8 * v[i] + 0.4035 * w[i] + 3.5 * vz[i]))
            if T_M5[i] < 0.05*T_hover[i]:
                T_M5[i] = 0.05*T_hover[i]
            ef_M5.append(1000/g/(cnst*(np.sqrt(T_M5[i]*g/(2*rho*A)))))
            Preq_M5.append(COAXIAL_80*(1000 * T_M5[i]/ef_M5[i]))
            
            T_M6.append(T_hover[i] + Cnst_PWM_T * (0.23 * v[i] + 0.4035 * w[i] + 3.5 * vz[i]))
            if T_M6[i] < 0.05*T_hover[i]:
                T_M6[i] = 0.05*T_hover[i]
            ef_M6.append(1000/g/(cnst*(np.sqrt(T_M6[i]*g/(2*rho*A)))))
            Preq_M6.append(COAXIAL_80*(1000 * T_M6[i]/ef_M6[i]))
            
            T_M7.append(T_hover[i] + Cnst_PWM_T * (- 0.23 * v[i] - 0.4035 * w[i] + 3.5 * vz[i]))
            if T_M7[i] < 0.05*T_hover[i]:
                T_M7[i] = 0.05*T_hover[i]
            ef_M7.append(1000/g/(cnst*(np.sqrt(T_M7[i]*g/(2*rho*A)))))
            Preq_M7.append(COAXIAL_80*(1000 * T_M7[i]/ef_M7[i]))
            
            T_M8.append(T_hover[i] + Cnst_PWM_T * (0.23 * v[i] - 0.4035 * w[i] + 3.5 * vz[i]))
            if T_M8[i] < 0.05*T_hover[i]:
                T_M8[i] = 0.05*T_hover[i]
            ef_M8.append(1000/g/(cnst*(np.sqrt(T_M8[i]*g/(2*rho*A)))))
            Preq_M8.append(COAXIAL_80*(1000 * T_M8[i]/ef_M8[i]))
            
            Preq_prop.append(Preq_M1[i] +  Preq_M2[i] + Preq_M3[i] + Preq_M4[i] + Preq_M5[i] + Preq_M6[i] + Preq_M7[i] + Preq_M8[i])
            
        elif n_motores == 6:
            
            T_M1.append(T_hover[i] + Cnst_PWM_T * (-0.06923890 * v[i] - 0.29355463 * w[i] + 2.70526734 * vz[i]))
            if T_M1[i] < 0.05*T_hover[i]:
                T_M1[i] = 0.05*T_hover[i]
            ef_M1.append(1000/g/(cnst*(np.sqrt(T_M1[i]*g/(2*rho*A)))))
            Preq_M1.append(1000 * T_M1[i]/ef_M1[i])
            
            T_M2.append(T_hover[i] +  Cnst_PWM_T * (-0.06923890 * v[i] + 0.29648913 * w[i] + 2.70526734 * vz[i]))
            if T_M2[i] < 0.05*T_hover[i]:
                T_M2[i] = 0.05*T_hover[i]
            ef_M2.append(1000/g/(cnst*(np.sqrt(T_M2[i]*g/(2*rho*A)))))
            Preq_M2.append(1000 * T_M2[i]/ef_M2[i])
            
            T_M3.append(T_hover[i] + Cnst_PWM_T * (-0.41846113 * v[i] - 0.29355463 * w[i] + 2.70526734 * vz[i]))
            if T_M3[i] < 0.05*T_hover[i]:
                T_M3[i] = 0.05*T_hover[i]
            ef_M3.append(1000/g/(cnst*(np.sqrt(T_M3[i]*g/(2*rho*A)))))
            Preq_M3.append(1000 * T_M3[i]/ef_M3[i])
            
            T_M4.append(T_hover[i] +  Cnst_PWM_T * (0.52299572 * v[i] + 0.29648913 * w[i] + 2.70526734 * vz[i]))
            if T_M4[i] < 0.05*T_hover[i]:
                T_M4[i] = 0.05*T_hover[i]
            ef_M4.append(1000/g/(cnst*(np.sqrt(T_M4[i]*g/(2*rho*A)))))
            Preq_M4.append(1000 * T_M4[i]/ef_M4[i])
    
            T_M5.append(T_hover[i] +  Cnst_PWM_T * (-0.41846113 * v[i] + 0.29648913 * w[i] + 2.70526734 * vz[i]))
            if T_M5[i] < 0.05*T_hover[i]:
                T_M5[i] = 0.05*T_hover[i]
            ef_M5.append(1000/g/(cnst*(np.sqrt(T_M5[i]*g/(2*rho*A)))))
            Preq_M5.append(1000 * T_M5[i]/ef_M5[i])
            
            T_M6.append(T_hover[i] +  Cnst_PWM_T * (0.52299572 * v[i] - 0.29355463 * w[i] + 2.70526734 * vz[i]))
            if T_M6[i] < 0.05*T_hover[i]:
                T_M6[i] = 0.05*T_hover[i]
            ef_M6.append(1000/g/(cnst*(np.sqrt(T_M6[i]*g/(2*rho*A)))))
            Preq_M6.append(1000 * T_M6[i]/ef_M6[i])
            
            Preq_prop.append(Preq_M1[i] +  Preq_M2[i] + Preq_M4[i] + Preq_M5[i] + Preq_M3[i] + Preq_M6[i])
            
        elif n_motores == 4:
            
            T_M1.append(T_hover[i] - 0.1548 * v[i] + 0.0423 * w[i] + 0.8315 * vz[i])
            if T_M1[i] < 0.05*T_hover[i]:
                T_M1[i] = 0.05*T_hover[i]
            ef_M1.append((1/(cnst * np.sqrt(T_M1[i]*g/(2*rho*A) ) )*1000/g))
            Preq_M1.append(1000 * T_M1[i]/ef_M1[i])
            
            T_M2.append(T_hover[i] - 0.1548 * v[i] - 0.0423 * w[i] + 0.8315 * vz[i])
            if T_M2[i] < 0.05*T_hover[i]:
                T_M2[i] = 0.05*T_hover[i]
            ef_M2.append((1/(cnst * np.sqrt(T_M2[i]*g/(2*rho*A) ) )*1000/g))
            Preq_M2.append(1000 * T_M2[i]/ef_M2[i])
            
            T_M3.append(T_hover[i] + 0.1548 * v[i] + 0.0423 * w[i] + 0.8315 * vz[i])
            if T_M3[i] < 0.05*T_hover[i]:
                T_M3[i] = 0.05*T_hover[i]
            ef_M3.append((1/(cnst * np.sqrt(T_M3[i]*g/(2*rho*A) ) )*1000/g))
            Preq_M3.append(1000 * T_M3[i]/ef_M3[i])
            
            T_M4.append(T_hover[i] + 0.1548 * v[i] - 0.0423 * w[i] + 0.8315 * vz[i])
            if T_M4[i] < 0.05*T_hover[i]:
                T_M4[i] = 0.05*T_hover[i]
            ef_M4.append((1/(cnst * np.sqrt(T_M4[i]*g/(2*rho*A) ) )*1000/g))
            Preq_M4.append(1000 * T_M4[i]/ef_M4[i])
            
            Preq_prop.append(Preq_M1[i] +  Preq_M2[i] + Preq_M3[i] + Preq_M4[i])
            
# CONSUMO DE TANQUE E BATERIA
#=============================================================================#  
        
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

        if M_pulv[i] - cons_pulv[i] < 0:
            M_pulv.append(0)
        else:
            M_pulv.append(M_pulv[i] - cons_pulv[i])
            
        if(STATUS[i] == "YAW+" or STATUS[i] == "YAW-" or STATUS[i] == "YAW-2"):
            t_manobra.append(t_manobra[i] + dt)
        else:
            t_manobra.append(t_manobra[i])
            
# OPERAÇÃO SEGUINTE - NOVOS PARÂMETROS
#=============================================================================#

        M_tot.append(M_vazio + M_pulv[i+1] + M_bat)
        E_bat.append(E_bat[i] - Preq_tot[i]*dt/3600)
        autonomia.append(3600*E_bat[i]/(Preq_tot[i]))            
        dist_percorr.append(dist_percorr[i] + math.sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2 + (z[i+1]-z[i])**2))
        dist_rtl.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
        t.append(t[i] + dt)
        voo_cor.append(voo)
        t_de_voo.append(t_de_voo[i] + dt)
        
# OPERAÇÃO SEGUINTE - RTL FIM
#=============================================================================#

        if (OP[i] == "RTL FIM"):
            if (x[i+1] == 0 and y[i+1] == 0 and z[i+1] == 0):
                OP.append("FIM")
                produtiv_por_voo.append(dist_pulv[i]*faixa)
                X_rtl_por_voo.append(x_rtl)
                Y_rtl_por_voo.append(y_rtl)
                t_voo.append(t_de_voo[i])
                Tempo_por_voo.append(t[len(t)-1])
                n_passada2 = n_passada2 + 1
                t[i+1] = t[i+1] + t_lavagem_limpeza + t_triplice_lavagem + t_desloc_pos_op
                M_retorno.append(M_pulv[i])
                Ebat_retorno.append(E_bat[i])
            else:
                OP.append("RTL FIM")
                
        elif((voo >= len(set_tanque)) and (OP[i] == "RTL BAT" or OP[i] == "RTL CALDA") and SETAR_TANQUE == "SIM") or (((x[i+1] >= math.ceil(X/faixa)*faixa + x0 - faixa/2) or x[i+1] >= max(x1) - faixa/2) and SETAR_TANQUE == "NAO") or (n_passada2 == n_passada and STATUS[i] != "YAW-" and y[i+1] >= y_max[-1]) or (n_passada2 == n_passada and STATUS[i] != "YAW+" and y[i+1] <= y_min[-1]):
            theta_rtl = 0
            alpha_ida = math.atan2(x[i+1],y[i+1])*180/math.pi
            if theta[i] == 0:
                alpha_volta = math.atan2(x[i+1],( y[i+1] + (v[i]**2)/(2*acel) ))*180/math.pi
            elif theta[i] == 180:
                alpha_volta = math.atan2(x[i+1],( y[i+1] - (v[i]**2)/(2*acel) ))*180/math.pi
            else:
                alpha_volta = math.atan2(x[i+1],(y[i+1]))*180/math.pi
            x_rtl = x[i+1]
            y_rtl = y[i+1]
            z_rtl = z_deslocando
            n_passada2 = 1 + n_passada2
            OP.append("RTL FIM")
            
            
# OPERAÇÃO SEGUINTE - RTL CALDA
#=============================================================================#
            
        elif(M_pulv[i+1] == 0 and (STATUS[i] != "YAW+" and STATUS[i] != "YAW-" )):
            if SETAR_POSICAO == "SIM":
                if iteracao_posicao == len(X_rtw)-1:
                    OP.append("RTL FIM")
            
            if (x[i+1] == 0 and y[i+1] == 0 and z[i+1] == 0):
                if theta[i+1] < 0:
                    theta[i+1] = theta[i+1] + 180 
                    
                if OTIMIZAR_TANQUE == "SIM":
                    M_Pulv = Otimizador_Tanque(E_bat_max, bateria_limite, M_pulv_max, M_bat, M_vazio, COAXIAL_80, g, 
                                              eta_escmotor, eta_helice, rho, A, n_motores, vazao, v_pulv, faixa, 
                                              x_rtl, y_rtl, theta_rtl, Y, v_desloc, zi, v_subida, fator_erro_otimizacao)
                    Massa_por_voo.append(M_Pulv)
                    M_pulv[i+1] = M_Pulv

                elif OTIMIZAR_TANQUE == "NAO":
                    if SETAR_TANQUE == "SIM":
                        iii +=1
                        M_Pulv[i+1] = set_tanque[iii]
                        Massa_por_voo.append(set_tanque[iii])
                    else:
                        M_Pulv[i+1] = M_pulv_max
                        Massa_por_voo.append(M_pulv_max)

                produtiv_por_voo.append(dist_pulv[i]*faixa)
                X_rtl_por_voo.append(x_rtl)
                Y_rtl_por_voo.append(y_rtl)
                t_voo.append(t_de_voo[i])
                Tempo_por_voo.append(t[len(t)-1])
                iteracao_posicao = iteracao_posicao + 1
                n_abs = n_abs + 1
                t[i+1] = t[i+1] + t_abs_calda
                M_Pulv = M_pulv_max              
                indice_voo.append(i+1) 
                E_bat[i+1] = E_bat_max
                M_retorno.append(M_pulv[i])
                Ebat_retorno.append(E_bat[i])
                voo = voo + 1
                OP.append("RTW")
                    
            elif (OP[i] == "PULVERIZANDO" or OP[i] == "DESLOCANDO"):
                theta_rtl = theta[i+1]
                alpha_ida = math.atan2(x[i+1],y[i+1])*180/math.pi
                if theta[i] == 0:
                    alpha_volta = math.atan2(x[i+1],( y[i+1] + (v[i]**2)/(2*acel) ))*180/math.pi
                elif theta[i] == 180:
                    alpha_volta = math.atan2(x[i+1],( y[i+1] - (v[i]**2)/(2*acel) ))*180/math.pi
                x_rtl = x[i+1]
                y_rtl = y[i+1]
                z_rtl = z_deslocando
                OP.append("RTL CALDA")
            else:
                OP.append("RTL CALDA")
                
# OPERAÇÃO SEGUINTE - RTL BAT
#=============================================================================#
            
        elif(OP[i] == "RTL BAT" or (E_bat[i] <= bateria_limite*E_bat[0] and STATUS[i] != "YAW+" and STATUS[i] !="YAW-")):
            if SETAR_POSICAO == "SIM":
                if iteracao_posicao == len(X_rtw)-1:
                    OP.append("RTL FIM")
                    
            if (x[i+1] == 0 and y[i+1] == 0 and z[i+1] == 0):
                if theta[i+1] < 0:
                    theta[i+1] = theta[i+1] + 180
                
                if OTIMIZAR_TANQUE == "SIM":
                    M_Pulv = Otimizador_Tanque(E_bat_max, bateria_limite, M_pulv_max, M_bat, M_vazio, COAXIAL_80, g, 
                                              eta_escmotor, eta_helice, rho, A, n_motores, vazao, v_pulv, faixa, 
                                              x_rtl, y_rtl, theta_rtl, Y, v_desloc, zi, v_subida, fator_erro_otimizacao)
                    Massa_por_voo.append(M_Pulv)
                    M_pulv[i+1] = M_Pulv

                elif OTIMIZAR_TANQUE == "NAO":
                    if SETAR_TANQUE == "SIM":
                        iii +=1
                        M_pulv[i+1] = set_tanque[iii]
                        Massa_por_voo.append(set_tanque[iii])
                    else:
                        M_pulv[i+1] = M_pulv_max
                        Massa_por_voo.append(M_pulv_max)
                  
                produtiv_por_voo.append(dist_pulv[i]*faixa)
                X_rtl_por_voo.append(x_rtl)
                Y_rtl_por_voo.append(y_rtl)
                t_voo.append(t_de_voo[i])
                Tempo_por_voo.append(t[len(t)-1])
                iteracao_posicao = iteracao_posicao + 1
                n_abs = n_abs + 1
                t[i+1] = t[i+1] + t_abs_calda
                M_Pulv = M_pulv_max              
                indice_voo.append(i+1) 
                E_bat[i+1] = E_bat_max
                M_retorno.append(M_pulv[i])
                Ebat_retorno.append(E_bat[i])
                voo = voo + 1
                OP.append("RTW")

            elif (OP[i] != "RTL BAT"):
                theta_rtl = theta[i+1]
                alpha_ida = math.atan2(x[i+1],y[i+1])*180/math.pi
                if theta[i] == 0:
                    alpha_volta = math.atan2(x[i+1],( y[i+1] + (v[i]**2)/(2*acel) ))*180/math.pi
                elif theta[i] == 180:
                    alpha_volta = math.atan2(x[i+1],( y[i+1] - (v[i]**2)/(2*acel) ))*180/math.pi
                x_rtl = x[i+1]
                y_rtl = y[i+1]
                z_rtl = z_deslocando
                OP.append("RTL BAT")
            else:
                OP.append("RTL BAT")
                
# OPERAÇÃO SEGUINTE - DESLOCANDO
#=============================================================================#

        elif(OP[i] == "DESLOCANDO"):
            if (x[i+1] == xi and y[i+1] ==  yi and z[i+1] == zi and theta[i+1] == thetai):
                if thetai == theta_dir:
                    OP.append("PULVERIZANDO")
                else:   
                    thetai = theta_dir
                    OP.append("DESLOCANDO")
            else:
                OP.append("DESLOCANDO")
                
# OPERAÇÃO SEGUINTE - RTW
#=============================================================================#

        elif(OP[i] == "RTW"):
            if (x[i+1] == x_rtl and y[i+1] ==  y_rtl and theta[i+1] == theta_rtl):
                OP.append("PULVERIZANDO")
            else:
                OP.append(OP[i])
        else:
            OP.append(OP[i])
            
# OPERAÇÃO SEGUINTE - PRÓXIMA ITERAÇÃO
#=============================================================================#
            
        i = i + 1
        
# FIM DA MISSÃO
#=============================================================================#
    
    if n_passada2 == (n_passada-1):
        STATUS.append("FIM")
        break
    else:
        break

# DESVIO PADRÃO DA DISTÂNCIA PERCORRIDA
#=============================================================================#

if SETAR_POSICAO == "SIM":           
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
        ErroX.append(100*(abs(X_rtl_por_voo[n-1]-X_rtw[n-1])/X_rtl_por_voo[n-1]))
        ErroY.append(100*(abs(Y_rtl_por_voo[n-1]-Y_rtw[n-1])/Y_rtl_por_voo[n-1]))
        dist_simulado = ((X_rtl_por_voo[n])**2+(Y_rtl_por_voo[n])**2)**(1/2)
        dist_ensaio =  ((X_rtw[n-1])**2+(Y_rtw[n-1])**2)**(1/2)
        ErroRTW.append((abs(dist_simulado-dist_ensaio)))
    ErroDistVooRel = []
    ErroDistVooAbs = []
    for n in range(len(Dist_voo_corrigida)):
        ErroDistVooRel.append(100*(abs(Dist_ensaio_voo[n]-Dist_voo_corrigida[n])/Dist_ensaio_voo[n]))
        ErroDistVooAbs.append(((Dist_ensaio_voo[n]-Dist_voo_corrigida[n])))    
    vol = []
    vetor_sem_ultimo = ErroDistVooAbs[:-1]
    desvio_rel_0 = np.sqrt(np.mean(np.array(vetor_sem_ultimo)**2))
    print("Desvio Padrão da Distância:",f"{desvio_rel_0:.0f}")
else:
    print("Finalizado")

# ANDAMENTO DA OPERAÇÃO POR VOO
#=============================================================================#

POR_VOO_Tempo = np.array(Tempo_por_voo) / 3600
POR_VOO_X_rtl = np.array(X_rtl_por_voo)
POR_VOO_Y_rtl = np.array(Y_rtl_por_voo)
POR_VOO_D_rtl = np.sqrt(POR_VOO_X_rtl**2 + POR_VOO_Y_rtl**2)  # Evita usar potência e simplifica
POR_VOO_Produtividade = np.array(produtiv_por_voo) / 10000
POR_VOO_Massa = np.array(Massa_por_voo)
POR_VOO_M_Retorno = np.array(M_retorno)
Ebat_retorno = [(x / E_bat[0]) * 100 for x in Ebat_retorno];POR_VOO_Ebat_Retorno = np.array(Ebat_retorno);
POR_VOO_T_VOO = np.array(t_voo) / 60
POR_VOO_T_VOO2 = [POR_VOO_T_VOO[0]] + [POR_VOO_T_VOO[i] - POR_VOO_T_VOO[i-1] for i in range(1, len(POR_VOO_T_VOO))]

andamento_por_voo = pd.DataFrame({
    "Voo": np.arange(1, len(POR_VOO_T_VOO2) + 1),  # Cria uma sequência de voos
    'Tempo Total [h]': POR_VOO_Tempo,
    'X RTL [m]': POR_VOO_X_rtl,
    'Y RTL [m]': POR_VOO_Y_rtl,
    'D RTL [m]': POR_VOO_D_rtl,
    'Bateria Retorno [%]': POR_VOO_Ebat_Retorno,
    'Tanque Saída [L]': POR_VOO_Massa,
    'Tanque Retorno [L]': POR_VOO_M_Retorno,
    'Produtividade [ha]': POR_VOO_Produtividade,
    'Tempo de Voo [min]': POR_VOO_T_VOO2})

# ANDAMENTO DA OPERAÇÃO DISCRETIZADO
#=============================================================================#
plt.figure(figsize=(10, 8))

# Retas geradas
plt.plot(x1, y1, 'o-', label="Linha 1 (x1, y1)", color='blue')
plt.plot(x2, y2, 's-', label="Linha 2 (x2, y2)", color='red')
plt.plot(x,y)
# Conexão entre os pontos
for i in range(len(x1)):
    plt.plot([x1[i], x2[i]], [y_min[i], y_max[i]], color='purple', linestyle='--', linewidth=0.8)

# Pontos fixos
plt.scatter([ponto1[0], ponto2[0], ponto3[0], ponto4[0]], 
            [ponto1[1], ponto2[1], ponto3[1], ponto4[1]], 
            color='green', label='Pontos Fixos', zorder=5)

# Nomes dos pontos
plt.text(ponto1[0], ponto1[1], 'P1', fontsize=12, ha='right', color='green')
plt.text(ponto2[0], ponto2[1], 'P2', fontsize=12, ha='right', color='green')
plt.text(ponto3[0], ponto3[1], 'P3', fontsize=12, ha='right', color='green')
plt.text(ponto4[0], ponto4[1], 'P4', fontsize=12, ha='right', color='green')

# Estilização
plt.title("Pontos e Retas Geradas")
plt.xlabel("x")
plt.ylabel("y")
plt.axhline(0, color='black', linewidth=0.5, linestyle='--')  # Eixo Y
plt.axvline(0, color='black', linewidth=0.5, linestyle='--')  # Eixo X
plt.legend()
plt.grid(True)
plt.show()


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
area_pulv_local = np.array(area_pulv_local)/10000

andamento_operacao = pd.DataFrame({
    'Tempo [s]': t,
    'X [m]': x,
    'Y [m]': y,
    'Z [m]': z,
    'Bateria [%]': E_bat,
    'Tanque [L]': M_pulv,
    'Vel. XY [m/s]': v,
    'Vel. Z [m/s]': -1*vz,
    'Theta [°]': -1*theta,
    'Produtividade [ha]': area_pulv_local,
    'Operação': OP,
    'Status': STATUS})

# PLOT 3D DA OPERAÇÃO
#=============================================================================#

intervalos_cores = [
(0, 'red'),(1, 'blue'),(2, 'green'),(3, 'purple'),(4, 'orange'),(5, 'yellow'),
(6, 'brown'),(7, 'pink'),(8, 'cyan'),(9, 'gray'),(10, 'black'),(11, 'magenta'),
(12, 'lime'),(13, 'teal'),(14, 'indigo'),(15, 'violet'),(16, 'gold'),
(17, 'olive'),(18, 'navy'),(19, 'maroon'),(20, 'turquoise'),(21, 'lavender'),
(22, 'khaki'),(23, 'salmon'),(24, 'coral'),(25, 'beige'),(26, 'crimson'),
(27, 'ivory'),(28, 'plum'),(29, 'aqua'),(30, 'chartreuse'),(31, 'sienna'),
(32, 'midnightblue'),(33, 'darkorange'),(34, 'darkgreen'),(35, 'lightpink'),
(36, 'goldenrod'),(37, 'darkviolet'),(38, 'steelblue'),(39, 'darkred'),
(40, 'slateblue'),(41, 'mediumorchid'),(42, 'lightseagreen'),(43, 'peachpuff'),
(44, 'lightcoral'),(45, 'deepskyblue'),(46, 'rosybrown'),(47, 'seagreen'),
(48, 'firebrick'),(49, 'dodgerblue'),(50, 'orangered')]

def atribuir_cor(valor_t):
    for intervalo, cor in intervalos_cores:
        if valor_t <= intervalo:
            return cor
    return intervalos_cores[-1][1]

cores = [atribuir_cor(valor) for valor in voo_cor]
fig_trajeto = go.Figure()
fig_trajeto.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', line=dict(color=cores, colorscale='Viridis', width=2),))
fig_trajeto.update_layout(scene=dict(aspectmode="cube"))
fig_trajeto.update_layout(paper_bgcolor="white", showlegend=False, scene=dict(aspectmode='data'))
fig_trajeto.write_html("3d_trajetória_simulado.html")