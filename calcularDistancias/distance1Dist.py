import openrouteservice
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# Não calculamos a variação do preço por aumento no numero distribuidora, pois a rota otimizada não suporta essa restição na capacidade de carga do caminhão

client = openrouteservice.Client(key="5b3ce3597851110001cf6248005f849ff0164b42b07ec43ef775a9fc", timeout=30)

# Coordenadas: Distribuidoras + Repúblicas
# Cada ponto: [longitude, latitude]
locais = [ 
    [-43.51009700172696, -20.403143454906736],  # Distribuidora 1
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
demandas = [0, 120, 170, 145, 150, 160, 70, 80, 80, 50, 140, 90, 120, 150, 60, 95, 60, 90, 140, 145, 170, 80, 150, 60, 145, 160, 65, 55, 90, 100, 95] 

def calcular_matriz_distancias(locais):
    response = client.distance_matrix(
        locations=locais,
        profile="driving-car",
        metrics=["distance"]
    )
    return response["distances"]

def resolver_otimizacao(matriz_distancias):
    """Resolve o problema de otimização sem restrição de capacidade."""
    manager = pywrapcp.RoutingIndexManager(len(matriz_distancias), 1, 0)
    routing = pywrapcp.RoutingModel(manager)
    
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(matriz_distancias[from_node][to_node])
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.time_limit.seconds = 30
    
    solution = routing.SolveWithParameters(search_parameters)
    
    if solution:
        rota_otimizada = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            rota_otimizada.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        rota_otimizada.append(manager.IndexToNode(index))
        return rota_otimizada
    else:
        return None

def gerar_rota_sequencial(locais, demandas, capacidade):
    """Gera uma rota sequencial com retornos à distribuidora mais próxima quando necessário."""
    matriz_distancias = calcular_matriz_distancias(locais)
    rota = [0]  # Começa na primeira distribuidora
    cumul = 0
    for i in range(1, len(locais)):
        if cumul + demandas[i] > capacidade:
            distancias = [matriz_distancias[i][j] for j in range(1)]
            melhor_distribuidora = distancias.index(min(distancias))
            rota.append(melhor_distribuidora)
            cumul = 0
        rota.append(i)
        cumul += demandas[i]
    rota.append(rota[0])  # Retorna à distribuidora no final
    return rota

def calcular_distancia_total(rota, matriz_distancias):
    distancia_total = 0
    for i in range(len(rota) - 1):
        origem = rota[i]
        destino = rota[i + 1]
        distancia_total += matriz_distancias[origem][destino]
    return distancia_total

rota_sequencial = gerar_rota_sequencial(locais, demandas, capacidade)
matriz_distancias = calcular_matriz_distancias(locais)
rota_otimizada = resolver_otimizacao(matriz_distancias)
def precoTotal(distancia): 
    precoTotal = (distancia / 2500) * 6.44 # Calculo do custo por uma viagem inteira
    return precoTotal

# Se quiser calcular para a rota sequencial + otimizada:
distancia_sequencial = calcular_distancia_total(rota_sequencial, matriz_distancias)
distanciaOtimizada = calcular_distancia_total(rota_otimizada, matriz_distancias)
print(f"Distância total da rota sequencial: {distancia_sequencial:.2f} metros")
print("")
print(f"Preço total pela viagem: R${precoTotal(distancia_sequencial):.2f}")
rota_sequencial = gerar_rota_sequencial(locais, demandas, capacidade)
print("")
print("Rota Sequencial:", rota_sequencial)
print("")
print(f"Distância total da rota otimizada: {distanciaOtimizada:.2f} metros")
print("")
print(f"Preço total pela viagem: R${precoTotal(distanciaOtimizada):.2f}")
print("")
print("Rota Otimizada:", rota_otimizada)