from flask import Flask, request, jsonify
import requests
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

app = Flask(__name__)

ORS_API_KEY = "5b3ce3597851110001cf6248f3380fa418534bd499a9945c9361973e"

def cluster_points(points, cluster_size=50):
    # Egyszerű szétosztás csoportokra, pl. pontok sorrendjében
    return [points[i:i + cluster_size] for i in range(0, len(points), cluster_size)]

def get_distance_matrix(coords):
    # coords = [(lat, lon), ...]
    locations = [[lon, lat] for lat, lon in coords]  # ORS: [lon, lat]

    url = "https://api.openrouteservice.org/v2/matrix/driving-car"
    headers = {
        "Authorization": ORS_API_KEY,
        "Content-Type": "application/json"
    }
    body = {
        "locations": locations,
        "metrics": ["distance"],
        "units": "m"
    }

    resp = requests.post(url, json=body, headers=headers)
    resp.raise_for_status()
    data = resp.json()
    return data["distances"]

def solve_tsp(distance_matrix):
    n = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def dist_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(dist_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Nem kötelező visszatérni a starthelyre:
    routing.SetFixedCostOfVehicle(0, 0)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.seconds = 10

    solution = routing.SolveWithParameters(search_params)
    if not solution:
        return []

    index = routing.Start(0)
    route = []
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    # Nem kell visszatérni a starthelyre, így nem tesszük hozzá az utolsó pontot

    return route

@app.route("/optimize", methods=["POST"])
def optimize_route():
    data = request.get_json()
    points = data.get("locations", [])
    if len(points) < 2:
        return jsonify({"error": "Legalább 2 koordináta szükséges."}), 400

    coords = [(float(p["lat"]), float(p["lng"])) for p in points]

    # Daraboljuk kisebb csomagokra
    clusters = cluster_points(coords, cluster_size=50)

    full_route = []
    offset = 0  # az összesített út indexeit kezeljük

    for cluster in clusters:
        dist_matrix = get_distance_matrix(cluster)
        route = solve_tsp(dist_matrix)
        if not route:
            return jsonify({"error": "Nem sikerült optimalizálni az egyik klasztert."}), 500
        # a klaszteren belüli indexeket globális indexszé alakítjuk
        global_route = [offset + i for i in route]
        full_route.extend(global_route)
        offset += len(cluster)

    return jsonify({"route_order": full_route})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
