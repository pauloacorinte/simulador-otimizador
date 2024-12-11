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

# Função para ajustar os pontos caso haja valores negativos
def ajustar_offset(pontos):
    # Encontrar o valor mínimo de x e y nos pontos rotacionados
    min_x = min(p[0] for p in pontos)
    min_y = min(p[1] for p in pontos)
    
    # Se o mínimo for negativo, adicionar o valor absoluto do mínimo como offset
    offset_x = 0 if min_x >= 0 else abs(min_x)
    offset_y = 0 if min_y >= 0 else abs(min_y)
    
    # Adicionar o offset em todos os pontos
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

pontos = [[50, 0], [70, 100], [100, 0], [100, 20]]

ponto1, ponto2, ponto3, ponto4 = ordenar_pontos(pontos)


# Entrada
thet = 0
faixa1 = 5
x1 = []
y1 = []
x2 = []
y2 = []
n_passada = 1
i = 0

ponto1_rotacionado = rotacionar_ponto(ponto1, thet)
ponto2_rotacionado = rotacionar_ponto(ponto2, thet)
ponto3_rotacionado = rotacionar_ponto(ponto3, thet)
ponto4_rotacionado = rotacionar_ponto(ponto4, thet)


pontos_rotacionados = [ponto1_rotacionado, ponto2_rotacionado, ponto3_rotacionado, ponto4_rotacionado]


pontos_ajustados, offset_x, offset_y = ajustar_offset(pontos_rotacionados)


ponto1, ponto2, ponto3, ponto4 = pontos_ajustados   


def calcular_reta(p1, p2):

    if p2[0] - p1[0] == 0:
        return 0, 0 
    else:
        coef_angular = (p2[1] - p1[1]) / (p2[0] - p1[0])
        coef_linear = p1[1] - coef_angular * p1[0]
        return coef_angular, coef_linear

coef12 = calcular_reta(ponto1, ponto2)
coef23 = calcular_reta(ponto2, ponto3)
coef34 = calcular_reta(ponto3, ponto4)
coef14 = calcular_reta(ponto1, ponto4)

max_x = max(p[0] for p in [ponto1, ponto2, ponto3, ponto4])

while True:
    # if ponto1[0] != ponto2[0]:
    if n_passada == 1:
        faixa = faixa1/2
    else :
        faixa = faixa1
    if coef12[0] >= 0:
        if n_passada == 1:
            x1.append(n_passada * faixa + ponto1[0])
        else:
            x1.append(n_passada * faixa + ponto1[0] - faixa/2)
        if x1[i] <= ponto4[0]:
            y1.append(coef14[0] * x1[i] + coef14[1])
        else:
            y1.append(coef34[0] * x1[i] + coef34[1])
    
    
        x2.append(x1[i])
        if x2[i] <= ponto2[0]:
            y2.append(coef12[0] * x2[i] + coef12[1])
        elif x2[i] > ponto3[0]:
            y2.append(coef34[0] * x2[i] + coef34[1])
        else:
            y2.append(coef23[0] * x2[i] + coef23[1])
            
    elif coef12[0] < 0:
        x1.append(n_passada * faixa + ponto2[0])
        if x1[i] <= ponto1[0]:
            y1.append(coef12[0] * x1[i] + coef12[1])
        elif x1[i] >= ponto4[0]:
            y1.append(coef34[0] * x1[i] + coef34[1])
        else:
            y1.append(coef14[0] * x1[i] + coef14[1])
    
        x2.append(x1[i])
        if ponto3[0] == max_x:
            y2.append(coef23[0] * x2[i] + coef23[1])
        else:
            if x2[i] <= ponto3[0]:
                y2.append(coef23[0] * x2[i] + coef23[1])
            else:
                y2.append(coef34[0] * x2[i] + coef34[1])
    if x1[i] >= (max_x-faixa/2):
        break
    n_passada += 1
    i += 1

plt.figure(figsize=(10, 8))

# Retas geradas
plt.plot(x1, y1, 'o-', label="Linha 1 (x1, y1)", color='blue')
plt.plot(x2, y2, 's-', label="Linha 2 (x2, y2)", color='red')

# Conexão entre os pontos
for i in range(len(x1)):
    plt.plot([x1[i], x2[i]], [y1[i], y2[i]], color='purple', linestyle='--', linewidth=0.8)

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

