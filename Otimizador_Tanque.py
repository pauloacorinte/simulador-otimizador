# -*- coding: utf-8 -*-
"""
Created on Thu Dec 12 10:21:32 2024

@author: EnricoAbreu
"""

import numpy as np

def Otimizador_Tanque(E_bat_max, bateria_limite, M_pulv_max, M_bat, M_vazio, COAXIAL_80, g, 
                          eta_escmotor, eta_helice, rho, A, n_motores, vazao, v_pulv, faixa, 
                          x_rtl, y_rtl, theta_rtl, Y, v_desloc, zi, v_subida, fator_erro_otimizacao):

    # Inicialização de variáveis
    energia_tanque = 0
    energia_ida = 0
    energia_voo = 0
    energia_volta = 0
    M_Pulv = 0

    # Loop para calcular a massa de pulverização
    while energia_voo <= E_bat_max * (1 - bateria_limite) and M_pulv_max >= M_Pulv + 0.5:
        M_Pulv += 0.5
        Massa_Total = M_bat + M_vazio + M_Pulv
        Massinha = np.linspace(M_bat + M_vazio, Massa_Total, 1000)
        
        # Cálculo da potência W
        W = COAXIAL_80 * Massinha * 1000 / ((1000 / g / (((1 / (eta_escmotor * eta_helice)) * 
            (np.sqrt(Massinha * g / (2 * rho * A * n_motores)))))))  # [W]
        
        # Energia para ida
        energia_ida = (COAXIAL_80 * Massa_Total * 1000 / ((1000 / g / (((1 / (eta_escmotor * eta_helice)) * 
            (np.sqrt(Massa_Total * g / (2 * rho * A * n_motores))))))) * 
            ((((x_rtl)**2 + (y_rtl)**2)**0.5) / v_desloc) / 3600)
        
        # Energia para tanque
        energia_tanque = np.trapz(W, Massinha) / (vazao * 60)  # [Wh]
        
        # Distância a ser percorrida
        distancia_p_percorrer = (M_Pulv / vazao) * v_pulv * 60
        
        # Cálculo de delta_x e delta_y com base em theta_rtl
        if theta_rtl == 0:
            if distancia_p_percorrer > Y - y_rtl:
                delta_x = (1 + int((distancia_p_percorrer - (Y - y_rtl)) / Y)) * faixa
                delta_y = ((distancia_p_percorrer - (Y - y_rtl)) / Y - int((distancia_p_percorrer - (Y - y_rtl)) / Y)) * Y
            else:
                delta_x = 0
                delta_y = distancia_p_percorrer
        elif theta_rtl == 180:
            if distancia_p_percorrer > y_rtl:
                delta_x = (1 + int((distancia_p_percorrer - y_rtl) / Y)) * faixa
                delta_y = ((distancia_p_percorrer - y_rtl) / Y - int((distancia_p_percorrer - y_rtl) / Y)) * Y
            else:
                delta_x = 0
                delta_y = distancia_p_percorrer
        
        # Ajuste de delta_y com base em delta_x/faixa
        if delta_x / faixa % 2 == 0:
            delta_y = np.cos(np.radians(theta_rtl)) * delta_y
        else:
            delta_y = np.cos(np.radians(theta_rtl + 180)) * delta_y
        
        # Cálculo da energia de subida, descida e curvas
        energia_subida = ((Massa_Total * g * zi) / 3600) + (zi / v_subida) * \
                         (COAXIAL_80 * Massa_Total * 1000 / ((1000 / g / (((1 / (eta_escmotor * eta_helice)) * 
                         (np.sqrt(Massa_Total * g / (2 * rho * A * n_motores))))))) / 3600)
        
        energia_descida = -((M_bat + M_vazio) * g * zi / 3600) + (zi / v_subida) * \
                          (COAXIAL_80 * (M_bat + M_vazio) * 1000 / ((1000 / g / (((1 / (eta_escmotor * eta_helice)) * 
                          (np.sqrt((M_bat + M_vazio) * g / (2 * rho * A * n_motores))))))) / 3600)
        
        energia_curva = ((Massa_Total * (v_pulv**2) / 2) / 3600)
        energia_curvas = energia_curva * delta_x / faixa
        
        # Energia total de voo
        energia_voo = (energia_tanque + energia_ida + energia_volta + energia_subida + energia_descida + energia_curvas) * fator_erro_otimizacao


    return M_Pulv

