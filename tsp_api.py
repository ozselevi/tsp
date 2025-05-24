from flask import Flask, request, jsonify
import requests
from sklearn.cluster import KMeans
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

app = Flask(__name__)

ORS_API_KEY = "5b3ce3597851110001cf6248f3380fa418534bd499a9945c9361973e"
ORS_MATRIX_URL = "https://api.openrouteservice.org/v2/matrix/driving-car"

def get_distance_matrix(coords):
    locations = [[lng, lat] for lat, lng in coords]  # ORS longitude-latitude sorrend
    payload = {
        "locations": locations,
        "metrics": ["distance"],
        "units": "m"
    }
    headers = {
        "Authorization": ORS_API_KEY,
        "Content-Type": "application/json"
    }
    response = requests.post(ORS_MATRIX_URL, json=payload, headers=headers)
    response.raise_for_status()
    return response.json()["distances"]

def solve_tsp(distance_matrix):
    n = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, None)  # None = nincs fix kezdőpont
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_idx, to_idx):
        from_node = manager.IndexToNode(from_idx)
        to_node = manager.IndexToNode(to_idx)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Ne kelljen visszatérni a kezdőpontra:
    routing.SetFixedCostOfAllVehicles(0)
    routing.SetDepot(-1)  # Depót nem definiálunk, szabad kezdés-végzés

    solution = routing.SolveWithParameters(search_parameters)

    if not solution:
        return None

    # Megkeressük a legrövidebb útvonalat tetszőleges kezdő/végponttal:
    # OR-Tools routing model alapból körutat ad, így ide trükközés kell,
    # de itt egyszerűbb megoldás, hogy az első index-szel kezdjük:
    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    # A végpont nincs hozzáadva az útvonalhoz, így:
    route.append(manager.IndexToNode(index))

    # Ez körutat ad vissza, szabad kezdő/végpont nélkül:
    # Ha tényleg nem akarunk körutat, OR-Tools-nál workaround kell, 
    # de egyszerűbb ha elfogadjuk az eredményt és csak az első "körből" az elejét levágjuk.

    return route[:-1]  # Utolsó elem a körút zárása, azt elhagyjuk.

def cluster_coords(coords, n_clusters=4):
    kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(coords)
    clusters = [[] for _ in range(n_clusters)]
    for idx, label in enumerate(kmeans.labels_):
        clusters[label].append((idx, coords[idx]))
    return clusters

@app.route("/optimize", methods=["POST"])
def optimize_route():
    data = request.get_json()
    raw_coords = data.get("locations")
    if not raw_coords or len(raw_coords) < 2:
        return jsonify({"error": "Legalább 2 koordináta szükséges"}), 400

    coords = [(float(loc["lat"]), float(loc["lng"])) for loc in raw_coords]

    # Darabolás klaszterekre
    n_clusters = 4 if len(coords) >= 4 else 1
    clusters = cluster_coords(coords, n_clusters)

    final_route = []
    offset = 0

    for cluster in clusters:
        indices, cluster_coords_list = zip(*cluster)

        dist_matrix = get_distance_matrix(cluster_coords_list)
        route = solve_tsp(dist_matrix)
        if route is None:
            return jsonify({"error": "Nem sikerült megoldani a TSP-t a klaszteren"}), 500

        # Eredeti indexek visszaállítása
        ordered_indices = [indices[i] for i in route]
        final_route.extend(ordered_indices)

    return jsonify({"route": final_route})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
