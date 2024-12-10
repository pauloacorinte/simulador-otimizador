# -*- coding: utf-8 -*-
"""
Created on Thu Feb  8 10:40:00 2024

ALTERAÇÃO 1 CICLO ->> 1 ABS

@author: MarcusMoreira
"""
import numpy as np
import math

def custos(M_pulv_max,capac_bat,area_tot,tempo_util,voo,dias,area_dia):
           # custos(20,8300,1.9835,17.6879,55,18,1000)

    valor_drone_carregador = -13.59*M_pulv_max**2 + 3159.4*M_pulv_max + 77592
    valor_bateria = 14578.05*np.log(capac_bat)-120979.66
    valor_eq_suporte = 35848
    valor_carro = 95000
    n_drones = 1
    dist_cidade_area = 150      #[km]
    dist_hotel_area = 35        #[km]
    n_periodos = dias*2
    
    # FISCAL -------------------#
    
    margem_lucro = 0.35
    ISS_simpes = 0.1209; ISS_presumido = 0.05; ISS_real = 0.05
    PIS = 0.0065; confins = 0.03; CSLL = 0.09; IRPJ = 0.15; base_lucro = 0.32
    contr_lucro = 0.5; outros_cpp = 0.05
    comissao_comercial = 0
    
    dias_uteis_ano = 5*52
    funcionarios = 2
    salario_piloto = 5500
    salario_auxiliar = 4500
    encargos = 0.63
    insalubridade = 1412 * 0.2
    hotel_diaria = 120
    alimentacao = 80
    telefone_mensal = 60
    EPI = 900
    vida_EPI = 6        #[meses]
    lavanderia_epi = 150; recorrencia = 14 #dias
    depreciacao_drone = 5   #[anos]
    depreciacao_eq = 7  #[anos]
    depreciacao_Carro = 5 #[anos]
    limp_sistema = 300; recorr_limp = 14 #dias
    limp_carro = 150; recorr_limp_carro = 14 #dias
    seguro_anual_drone = 550
    taxa_outros = 0.15 ; ocorrencia = 3 # manuntenção por ano
    seguro_anual_carro = 4000; impostos_carro = 1300
    outros_anual = 4500
    
#================# PREÇO POR DIA # =============================#

    custo_folha_dia_piloto = (salario_piloto*(1+encargos) + insalubridade)/(dias_uteis_ano/12)
    custo_folha_dia_auxilar = (salario_auxiliar*(1+encargos) + insalubridade)/(dias_uteis_ano/12)
    custo_hotel_dia = hotel_diaria * funcionarios
    custo_alimentacao_dia = alimentacao*funcionarios
    custo_telefone_dia = telefone_mensal/(dias_uteis_ano/12)
    custo_EPI_dia = (EPI/vida_EPI)*funcionarios/(dias_uteis_ano/12)
    custo_lavan_EPI_dia = (lavanderia_epi/recorrencia)*funcionarios
    custo_depr_drone_dia = valor_drone_carregador/depreciacao_drone/dias_uteis_ano
    custo_depr_eq_dia = valor_eq_suporte/(depreciacao_eq*dias_uteis_ano)
    custo_limp_sistema_dia = limp_sistema/recorr_limp
    custo_seguro_drone_dia = seguro_anual_drone/dias_uteis_ano
    custos_outros_dia = ocorrencia * taxa_outros * valor_drone_carregador/dias_uteis_ano
    custo_depre_carro_dia = valor_carro/depreciacao_Carro/dias_uteis_ano
    custo_limp_carro_dia = limp_carro/recorr_limp_carro
    custo_seguro_carro_dia = seguro_anual_carro/dias_uteis_ano
    custos_imposto_carro_dia = impostos_carro/dias_uteis_ano
    cutos_outros_dia = outros_anual/dias_uteis_ano
    
    custos_dia = custo_folha_dia_piloto + custo_folha_dia_auxilar + custo_hotel_dia + custo_alimentacao_dia + custo_telefone_dia + custo_EPI_dia + custo_lavan_EPI_dia + custo_depr_drone_dia + custo_depr_eq_dia + custo_limp_sistema_dia + custo_seguro_drone_dia + custos_outros_dia + custo_depre_carro_dia + custo_limp_carro_dia + custo_seguro_carro_dia + custos_imposto_carro_dia + cutos_outros_dia

    preco_servico_dia_simples = custos_dia/(1 - margem_lucro - ISS_simpes)
    preco_servico_dia_presumido = custos_dia/(1-(2*ISS_presumido + PIS + confins + base_lucro*(CSLL+IRPJ) + outros_cpp + margem_lucro))
    preco_servico_dia_real = custos_dia/(1-(2*ISS_real + PIS + confins + contr_lucro*margem_lucro*(CSLL+IRPJ) + outros_cpp + margem_lucro))


#================# PREÇO POR km # =============================#

    vida_carro = 250000                         #[km]
    custo_vida_km = valor_carro/vida_carro
    combustivel = 6             #[R$/L]
    consumo_carro = 9           #[km/L]
    custo_comb_km = combustivel/consumo_carro       #[R$/km]
    custo_pedagio_km = 0.1          #[R$/km]
    custo_revisao1 = 500
    intervalo_revisao1 = 5000       #[km]
    custo_revisa2 = 1500
    intervalo_revisao2 = 20000      #[km]
    custo_revisao3 = 4500
    intervalo_revisao3 = 40000
    custo_revisao_km = custo_revisao1/intervalo_revisao1 + custo_revisa2/intervalo_revisao2 + custo_revisao3/intervalo_revisao3
    custo_km_rodado = custo_vida_km + custo_comb_km + custo_pedagio_km + custo_revisao_km
    
    preco_servico_km_simples = custo_km_rodado/(1 - margem_lucro - ISS_simpes)
    preco_servico_km_presumido = custo_km_rodado/(1-(2*ISS_presumido + PIS + confins + base_lucro*(CSLL+IRPJ) + outros_cpp + margem_lucro))
    preco_servico_km_real = custo_km_rodado/(1-(2*ISS_real + PIS + confins + contr_lucro*margem_lucro*(CSLL+IRPJ) + outros_cpp + margem_lucro))

#=================# PREÇO POR ha #===============================#

    vida_drone_carregador = 6000           #[horas de voo]
    custo_vida_drone_carregador_ha = valor_drone_carregador*tempo_util/(vida_drone_carregador*area_dia)
    vida_bateria = 1500     #[ciclos]
    custo_bateria =  valor_bateria*voo/(vida_bateria)         #[R$/ha]
    media_revisao1 = 0.04
    revisao_drone1_ha = 800*area_dia/tempo_util    #[ha]
    media_revisao2 = 0.08
    revisao_drone2_ha = 1080*area_dia/tempo_util   #[ha]
    custo_revisao_ha = valor_drone_carregador*(media_revisao1/revisao_drone1_ha + media_revisao2/revisao_drone2_ha)
    
    comissao_piloto = 0.05
    custo_comissao_piloto_ha_simples = 0.0
    custo_comissao_piloto_ha_presumido = 0.0 
    custo_comissao_piloto_ha_real = 0.0
    flag1 = 0.05 * 20; flag2 = 0.05 * 20; flag3  = 0.05 * 20
    ERRO = 1
    
    while (ERRO > 10**-10):
        custo_comissao_piloto_ha_simples = flag1
        custo_comissao_piloto_ha_presumido = flag2
        custo_comissao_piloto_ha_real = flag3
        
        custo_por_ha_simples = custo_vida_drone_carregador_ha + custo_bateria   + custo_revisao_ha + custo_comissao_piloto_ha_simples
        custo_por_ha_presumido = custo_vida_drone_carregador_ha + custo_bateria  + custo_revisao_ha + custo_comissao_piloto_ha_presumido
        custo_por_ha_real = custo_vida_drone_carregador_ha + custo_bateria  + custo_revisao_ha + custo_comissao_piloto_ha_real
        
        preco_servico_ha_simples = custo_por_ha_simples/(1 - margem_lucro - ISS_simpes)
        preco_servico_ha_presumido = custo_por_ha_presumido/(1-(2*ISS_presumido + PIS + confins + base_lucro*(CSLL+IRPJ) + outros_cpp + margem_lucro))
        preco_servico_ha_real = custo_por_ha_real/(1-(2*ISS_real + PIS + confins + contr_lucro*margem_lucro*(CSLL+IRPJ) + outros_cpp + margem_lucro))
        
        preço_total_simples = (dias*preco_servico_dia_simples*(1 + comissao_comercial)*n_drones) + preco_servico_km_simples*(2*dist_cidade_area + n_periodos*dist_hotel_area) + preco_servico_ha_simples*area_tot*(1 + comissao_comercial)*n_drones
        preço_total_presumido = (dias*preco_servico_dia_presumido*(1 + comissao_comercial)*n_drones) + preco_servico_km_presumido*(2*dist_cidade_area + n_periodos*dist_hotel_area) + preco_servico_ha_presumido*area_tot*(1 + comissao_comercial)*n_drones
        preço_total_real = (dias*preco_servico_dia_real*(1 + comissao_comercial)*n_drones) + preco_servico_km_real*(2*dist_cidade_area + n_periodos*dist_hotel_area) + preco_servico_ha_real*area_tot*(1 + comissao_comercial)*n_drones
        
        preço_total_por_ha_simples = preço_total_simples/area_tot
        preço_total_por_ha_presumido = preço_total_presumido/area_tot
        preço_total_por_ha_real = preço_total_real/area_tot
        
        flag1 = comissao_piloto*preço_total_por_ha_simples
        flag2 = comissao_piloto*preço_total_por_ha_presumido
        flag3 = comissao_piloto*preço_total_por_ha_real
        ERRO = abs(custo_comissao_piloto_ha_simples-flag1 + custo_comissao_piloto_ha_presumido - flag2 + custo_comissao_piloto_ha_real -flag3)
        
        # print(custo_comissao_piloto_ha_simples,flag1,custo_comissao_piloto_ha_presumido,flag2,custo_comissao_piloto_ha_real,flag3)
        
        #print(preço_total_por_ha_simples,preço_total_por_ha_presumido,preço_total_por_ha_real)
        #return(custo_revisao_ha*1000,(custo_gasol_drone_ha+custo_oleo_drone_ha)*1000,custo_comissao_piloto_ha_real*1000,custo_bateria*1000,vida_drone_carregador*1000,(2*dist_cidade_area + n_periodos*dist_hotel_area)*custo_revisao_km,(2*dist_cidade_area + n_periodos*dist_hotel_area)*custo_pedagio_km,(2*dist_cidade_area + n_periodos*dist_hotel_area)*custo_comb_km,(2*dist_cidade_area + n_periodos*dist_hotel_area)*custo_vida_km,cutos_outros_dia*dias,custos_imposto_carro_dia*dias,custo_seguro_carro_dia*dias,custo_limp_carro_dia*dias,custo_depre_carro_dia*dias,custos_outros_dia*dias,custo_seguro_drone_dia*dias,custo_limp_sistema_dia*dias,custo_depr_eq_dia*dias,custo_depr_drone_dia*dias,custo_lavan_EPI_dia*dias,custo_EPI_dia*dias,custo_telefone_dia*dias,custo_alimentacao_dia*dias,custo_hotel_dia*dias,custo_folha_dia_auxilar*dias,custo_folha_dia_piloto*dias,preço_total_simples,preço_total_presumido,preço_total_real,preço_total_por_ha_simples,preço_total_por_ha_presumido,preço_total_por_ha_real)
        return(preço_total_por_ha_real,valor_drone_carregador + valor_bateria)
    
# a = custos(20,8300,1.9835,17.6879,55,18,1000)