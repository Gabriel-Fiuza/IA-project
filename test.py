import openrouteservice
import folium
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

client = openrouteservice.Client(key="5b3ce3597851110001cf6248005f849ff0164b42b07ec43ef775a9fc", timeout=30)

# Coordenadas: Distribuidoras + Repúblicas
# Cada ponto: [longitude, latitude]
locais = [ 
    [-43.51009700172696, -20.403143454906736],  # Distribuidora 1
    [-43.508240825919664, -20.399392937048777],  # Distribuidora 2
    [-43.503704233635, -20.386127763439696],  # Distribuidora 3 
    [-43.50712520607496, -20.388796690003044],  # Distribuidora 4
    [-43.50132175877035, -20.391887176489956],
    [-43.49912620784418, -20.394167499318744],  
    [-43.50841549007973, -20.40111109613239],
    [-43.51338062132738, -20.398889328822563],
    [-43.5072875593975, -20.400356039093445],
    [-43.510609940362485, -20.3993376733433],
    [-43.50898445434185, -20.402751323014655],
    [-43.51147152317416, -20.40379880132609],
    [-43.503136534821415, -20.39940301744866],
    [-43.51355344025896, -20.398947665039916],
    [-43.514857663091604, -20.396453149551817],
    [-43.502589774738766, -20.39716201263336],
    [-43.507235553779395, -20.40143121070148],
    [-43.508012877710605, -20.400432119830473],
    [-43.50949541461815, -20.403129126503366],
    [-43.50672596124444, -20.39947733699732],
    [-43.5097003693425, -20.39934943996829],
    [-43.507691594338425, -20.399000343283724],
    [-43.50731060700543, -20.39921397081579],
    [-43.50954934684449, -20.39993452392748],
    [-43.507514739492, -20.387616050123114],
    [-43.50246138251594, -20.388267980737275],
    [-43.505952119264826, -20.385586226346508],
    [-43.502301958552884, -20.388734272524314],
    [-43.50167681821058, -20.387405976474252],
    [-43.505306974319936, -20.38531505501014],
    [-43.507722391839486, -20.386922642076584],
    [-43.511020996826666, -20.40387553416929],
    [-43.50959836714495, -20.403245396346804],
    [-43.502563114453004, -20.384316352256896],
]

# Parâmetros de capacidade e demandas
capacidade = 500
# As quatro primeiras posições são distribuidoras e têm demanda 0
demandas = [0, 0, 0, 0, 120, 170, 145, 150, 160, 70, 80, 80, 50, 140, 90, 120, 150, 60, 95, 60, 90, 140, 145, 170, 80, 150, 60, 145, 160, 65, 55, 90, 100, 95] 

def calcular_matriz_distancias(locais):
    response = client.distance_matrix(
        locations=locais,
        profile="driving-car",
        metrics=["distance"]
    )
    return response["distances"]

def gerar_rota_sequencial(locais, demandas, capacidade):
    """Gera uma rota sequencial com retornos à distribuidora mais próxima quando necessário."""
    matriz_distancias = calcular_matriz_distancias(locais)
    rota = [0]  # Começa na primeira distribuidora
    cumul = 0
    for i in range(4, len(locais)):
        if cumul + demandas[i] > capacidade:
            distancias = [matriz_distancias[i][j] for j in range(4)]
            melhor_distribuidora = distancias.index(min(distancias))
            rota.append(melhor_distribuidora)
            cumul = 0
        rota.append(i)
        cumul += demandas[i]
    rota.append(rota[0])  # Retorna à distribuidora no final
    return rota

def otimizar_rota_com_gls(matriz_distancias):
    num_locais = len(matriz_distancias)

    # Criando o gerenciador de índice dos nós
    gerenciador = pywrapcp.RoutingIndexManager(num_locais, 1, 0)

    # Criando o modelo de roteamento
    roteador = pywrapcp.RoutingModel(gerenciador)

    # Definindo a função de custo (distância entre locais)
    def distancia_callback(from_index, to_index):
        from_node = gerenciador.IndexToNode(from_index)
        to_node = gerenciador.IndexToNode(to_index)
        return matriz_distancias[from_node][to_node]

    transit_callback_index = roteador.RegisterTransitCallback(distancia_callback)
    roteador.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Definindo os parâmetros de busca com Guided Local Search
    parametros_busca = pywrapcp.DefaultRoutingSearchParameters()
    parametros_busca.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.Value("PATH_CHEAPEST_ARC")
    parametros_busca.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.Value("GUIDED_LOCAL_SEARCH")
    parametros_busca.time_limit.seconds = 10  # Tempo máximo de execução

    # Resolvendo o problema
    solucao = roteador.SolveWithParameters(parametros_busca)

    if solucao:
        rota_otimizada = []
        index = roteador.Start(0)
        while not roteador.IsEnd(index):
            rota_otimizada.append(gerenciador.IndexToNode(index))
            index = solucao.Value(roteador.NextVar(index))
        rota_otimizada.append(gerenciador.IndexToNode(index))  # Retorna ao depósito
        return rota_otimizada
    else:
        print("Nenhuma solução encontrada.")
        return None

def gerar_mapa(rota, nome_arquivo):
    """Gera um mapa com a rota fornecida e salva como HTML."""
    mapa = folium.Map(location=[-20.403143454906736, -43.51009700172696], zoom_start=14)
    for idx in rota:
        cor = "red" if idx < 4 else "green"
        folium.Marker(location=[locais[idx][1], locais[idx][0]], icon=folium.Icon(color=cor), popup=f"Índice: {idx}").add_to(mapa)
    rota_coords = [locais[i] for i in rota]
    directions = client.directions(coordinates=rota_coords, profile="driving-car", format="geojson")
    folium.GeoJson(directions, name="Rota").add_to(mapa)
    mapa.save(nome_arquivo)
    print(f"Mapa salvo como '{nome_arquivo}'")

rota_sequencial = gerar_rota_sequencial(locais, demandas, capacidade)
print("Rota Sequencial:", rota_sequencial)
gerar_mapa(rota_sequencial, "rota_sequencial.html")

matriz_distancias = calcular_matriz_distancias(locais)
rota_otimizada = otimizar_rota_com_gls(matriz_distancias)
print("Rota Otimizada:", rota_otimizada)
if rota_otimizada:
    gerar_mapa(rota_otimizada, "rota_otimizada.html")
else:
    print("Nenhuma solução otimizada encontrada.")
