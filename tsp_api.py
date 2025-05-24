from flask import Flask, request, jsonify
import requests
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from sklearn.cluster import KMeans
import numpy as np

app = Flask(__name__)

ORS_API_KEY = "5b3ce3597851110001cf6248f3380fa418534bd499a9945c9361973e"

def get_distance_matrix(coords):
    # ORS-nek lon-lat kell
    locations = [[lon, lat] for lat, lon in coords]

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

    # Nem kell visszatérni a kezdőpontra:
    routing.SetFixedCostOfVehicle(0, 0)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.seconds = 30  # idő korlát növelhető

    solution = routing.SolveWithParameters(search_params)
    if not solution:
        return []

    index = routing.Start(0)
    route = []
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    return route

def cluster_points(points, n_clusters=4):
    coords_np = np.array(points)
    kmeans = KMeans(n_clusters=n_clusters, random_state=42).fit(coords_np)
    labels = kmeans.labels_
    centers = kmeans.cluster_centers_

    clusters = [[] for _ in range(n_clusters)]
    for point, label in zip(points, labels):
        clusters[label].append(point)
    return clusters, centers

@app.route("/optimize", methods=["POST"])
def optimize_route():
    data = request.get_json()
    points = data.get("locations", [])
    if len(points) < 2:
        return jsonify({"error": "Legalább 2 koordináta szükséges."}), 400

    # Tisztítsd a koordinátákat, konvertálj float-ra, üres értékeket hagyj ki
    coords = []
    for p in points:
        try:
            lat = float(p["lat"])
            lng = float(p["lng"])
            coords.append((lat, lng))
        except (KeyError, ValueError, TypeError):
            continue

    if len(coords) < 2:
        return jsonify({"error": "Legalább 2 érvényes koordináta szükséges."}), 400

    # 1. Klaszterezés
    n_clusters = 4 if len(coords) >= 4 else 1
    clusters, centers = cluster_points(coords, n_clusters)

    # 2. Klaszterközpontok sorrendjének optimalizálása (kicsi TSP)
    # Egyszerű Euclid távolság a klaszterközpontok között:
    def euclidean_dist(a, b):
        return int(np.linalg.norm(np.array(a) - np.array(b)) * 1000)  # méterben
