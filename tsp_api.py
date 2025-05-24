from flask import Flask, request, jsonify
import numpy as np
from sklearn.cluster import KMeans
import requests
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

app = Flask(__name__)

ORS_API_KEY = "5b3ce3597851110001cf6248f3380fa418534bd499a9945c9361973e"
ORS_MATRIX_URL = "https://api.openrouteservice.org/v2/matrix/driving-car"

def fetch_distance_matrix(locations):
    """Lekéri az ORS távolságmátrixot közúton (méterben)."""
    coords = [[lng, lat] for lat, lng in locations]  # ORS: [lon, lat]
    data = {
        "locations": coords,
        "metrics": ["distance"],
        "units": "m"
    }
    headers = {
        "Authorization": ORS_API_KEY,
        "Content-Type": "application/json"
    }
    response = requests.post(ORS_MATRIX_URL, json=data, headers=headers)
    response.raise_for_status()
    matrix = response.json()["distances"]
    return matrix

def solve_tsp(distance_matrix):
    """OR-Tools TSP megoldás (nem kell visszatérni a startpontra)."""
    n = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Nem kötelező visszatérni a kiindulópontra
    routing.AddConstraint(
        routing.solver().MakeNoCycle(routing.NextVar(0))
    )
    # vagy alternatív megoldás: routing.SetDepot(-1), de ez nem mindig működik jól

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        route = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        # Nem zárjuk a kört vissza a startpontra
        return route
    else:
        return None

@app.route("/optimize", methods=["POST"])
def optimize():
    data = request.get_json()
    raw_locations = data.get("locations", [])

    coords = [
        (float(loc["lat"]), float(loc["lng"]))
        for loc in raw_locations
        if loc.get("lat") not in [None, ""] and loc.get("lng") not in [None, ""]
    ]

    if len(coords) < 2:
        return jsonify({"error": "Legalább 2 koordináta szükséges."}), 400

    # Klaszterezés (4 klaszter)
    k = 4 if len(coords) >= 4 else 1
    kmeans = KMeans(n_clusters=k, random_state=42).fit(coords)
    labels = kmeans.labels_

    clusters = [[] for _ in range(k)]
    for i, label in enumerate(labels):
        clusters[label].append((i, coords[i]))  # index és koordináta

    final_route = []
    cluster_centers = kmeans.cluster_centers_

    # Egyszerű klasztersorrend: klaszterközéppontok sorrendje (pl. északról délre)
    sorted_clusters_order = np.argsort(cluster_centers[:, 0])  # lat szerint rendezve, módosítható

    for ci in sorted_clusters_order:
        cluster = clusters[ci]
        if not cluster:
            continue

        indices, cluster_coords = zip(*cluster)

        # Lekérjük a távolságmátrixot klaszteren belül
        matrix = fetch_distance_matrix(cluster_coords)
        route = solve_tsp(matrix)
        if route is None:
            return jsonify({"error": "Nem sikerült TSP megoldást találni."}), 500

        # A klaszteren belüli pontok eredeti globális indexe
        ordered_indices = [indices[i] for i in route]
        final_route.extend(ordered_indices)

    return jsonify({"route": final_route})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
