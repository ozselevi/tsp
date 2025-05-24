from flask import Flask, request, jsonify
import requests
from sklearn.cluster import KMeans
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

app = Flask(__name__)

ORS_API_KEY = "5b3ce3597851110001cf6248f3380fa418534bd499a9945c9361973e"

def get_distance_matrix(coords):
    """
    Lekéri az OpenRouteService távolságmátrixot közútra.
    coords: [(lat, lng), ...]
    Visszaadja: NxN mátrix méterben.
    """
    locations = [[lng, lat] for lat, lng in coords]  # ORS azonos formátum (lon, lat)
    url = "https://api.openrouteservice.org/v2/matrix/driving-car"

    headers = {
        "Authorization": ORS_API_KEY,
        "Content-Type": "application/json"
    }
    data = {
        "locations": locations,
        "metrics": ["distance"],
        "units": "m"
    }

    resp = requests.post(url, json=data, headers=headers)
    if resp.status_code != 200:
        raise Exception(f"ORS hiba: {resp.status_code} - {resp.text}")

    matrix = resp.json()["distances"]
    return matrix

def solve_tsp(distance_matrix):
    n = len(distance_matrix)
    start = 0  # Fix kezdőpont

    manager = pywrapcp.RoutingIndexManager(n, 1, start)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_idx, to_idx):
        from_node = manager.IndexToNode(from_idx)
        to_node = manager.IndexToNode(to_idx)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Nem kell visszatérni a kezdőpontra (open route)
    routing.SetFixedCostOfAllVehicles(0)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)

    if not solution:
        return None

    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    route.append(manager.IndexToNode(index))

    return route

def cluster_coords(coords, n_clusters=4):
    """
    Pontokat klaszterez KMeans-szel.
    Visszaadja klaszterek listáját, ahol
    egy klaszter [(eredeti_index, (lat,lng)), ...]
    """
    if len(coords) <= n_clusters:
        return [[(i, coords[i])] for i in range(len(coords))]

    kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(coords)
    labels = kmeans.labels_

    clusters = [[] for _ in range(n_clusters)]
    for i, label in enumerate(labels):
        clusters[label].append((i, coords[i]))

    return clusters

@app.route("/optimize", methods=["POST"])
def optimize_route():
    data = request.get_json()
    raw_coords = data.get("locations")
    if not raw_coords or len(raw_coords) < 2:
        return jsonify({"error": "Legalább 2 koordináta szükséges"}), 400

    coords = [(float(loc["lat"]), float(loc["lng"])) for loc in raw_coords]

    # Klaszterezés
    n_clusters = 4 if len(coords) >= 4 else 1
    clusters = cluster_coords(coords, n_clusters)

    final_route = []
    visited = set()

    try:
        for cluster in clusters:
            indices, cluster_coords_list = zip(*cluster)

            dist_matrix = get_distance_matrix(cluster_coords_list)
            route = solve_tsp(dist_matrix)
            if route is None:
                return jsonify({"error": "Nem sikerült megoldani a TSP-t a klaszteren"}), 500

            ordered_indices = [indices[i] for i in route]

            for idx in ordered_indices:
                if idx not in visited:
                    final_route.append(idx)
                    visited.add(idx)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

    return jsonify({"route": final_route})
@app.route("/optimize", methods=["POST"])
def optimize_route():
    data = request.get_json()
    raw_coords = data.get("locations")
    if not raw_coords or len(raw_coords) < 2:
        return jsonify({"error": "Legalább 2 koordináta szükséges"}), 400

    coords = [(float(loc["lat"]), float(loc["lng"])) for loc in raw_coords]

    # Klaszterezés
    n_clusters = 4 if len(coords) >= 4 else 1
    clusters = cluster_coords(coords, n_clusters)

    final_route = []
    visited = set()

    try:
        for cluster in clusters:
            indices, cluster_coords_list = zip(*cluster)

            dist_matrix = get_distance_matrix(cluster_coords_list)
            route = solve_tsp(dist_matrix)
            if route is None:
                return jsonify({"error": "Nem sikerült megoldani a TSP-t a klaszteren"}), 500

            ordered_indices = [indices[i] for i in route]

            for idx in ordered_indices:
                if idx not in visited:
                    final_route.append(idx)
                    visited.add(idx)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

    return jsonify({"route": final_route})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
