# -*- coding: utf-8 -*-
"""
Created on Wed Dec 11 13:20:17 2024

@author: PauloAcorinte
"""

import math
import matplotlib.pyplot as plt

# Função para rotacionar um ponto
def rotacionar_ponto(ponto, theta):
    # Convertendo theta para radianos
    theta_rad = math.radians(theta)
    
    # Calculando as novas coordenadas
    x_novo = ponto[0] * math.cos(theta_rad) - ponto[1] * math.sin(theta_rad)
    y_novo = ponto[0] * math.sin(theta_rad) + ponto[1] * math.cos(theta_rad)
    
    return [x_novo, y_novo]

def ajustar_offset(pontos):
    min_x = min(p[0] for p in pontos)
    min_y = min(p[1] for p in pontos)

    offset_x = 0 if min_x >= 0 else abs(min_x)
    offset_y = 0 if min_y >= 0 else abs(min_y)

    pontos_ajustados = [[p[0] + offset_x, p[1] + offset_y] for p in pontos]
    
    return pontos_ajustados, offset_x, offset_y

def distancia_origem(ponto):
    return math.sqrt(ponto[0]**2 + ponto[1]**2)

def angulo_em_relaçao_ao_eixo_x(ponto):
    return math.atan2(ponto[1], ponto[0])

def ordenar_pontos(pontos):

    ponto1 = min(pontos, key=distancia_origem)

    pontos_restantes = [p for p in pontos if p != ponto1]

    ponto2 = max(pontos_restantes, key=angulo_em_relaçao_ao_eixo_x)

    pontos_restantes.remove(ponto2)

    ponto3 = max(pontos_restantes, key=lambda p: p[1])

    ponto4 = [p for p in pontos_restantes if p != ponto3][0]

    return ponto1, ponto2, ponto3, ponto4

def calcular_reta(p1, p2):

    if p2[0] - p1[0] == 0:
        return 0, 0 
    else:
        coef_angular = (p2[1] - p1[1]) / (p2[0] - p1[0])
        coef_linear = p1[1] - coef_angular * p1[0]
        return coef_angular, coef_linear

