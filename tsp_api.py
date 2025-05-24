from flask import Flask, request, jsonify
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import os
import requests

app = Flask(__name__)

ORS_API_KEY = "ITT_ÍRD_BE_A_SAJÁT_ORS_API_KULCSODAT"

def get_distance_matrix(locations):
    """
    Közúti távolságmátrix lekérése az ORS API-n keresztül.
    locations: [(lat, lng), ...]
    """
    coords_str = "|".join([f"{lng},{lat}" for lat, lng in locations])  # ORS lng,lat sorrend!
    url = f"https://api.openrouteservice.org/v2/matrix/driving-car"
    headers = {
        "Authorization": ORS_API_KEY,
        "Content-Type": "application/json"
    }
    json_data = {
        "locations": [[lng, lat] for lat, lng in locations],
        "metrics": ["distance"],
        "units": "m"
    }
    response = requests.post(url, json=json_data, headers=headers)
    if response.status_code != 200:
        raise Exception(f"ORS API hiba: {response.status_code} {response.text}")
    data = response.json()
    return data["distances"]

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
        return jsonify({"error": "Legalább 2 érvényes koordináta szükséges."}), 400

    try:
        distance_matrix = get_distance_matrix(coords)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, 0)  # 1 jármű, fix kezdőpont (0)
    routing = pywrapcp.RoutingModel(manager)

    def dist_callback(from_idx, to_idx):
        return int(distance_matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)])

    transit_index = routing.RegisterTransitCallback(dist_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_index)

    # Nincs visszatérés a kezdőpontra:
    routing.SetDepot(-1)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)

    if not solution:
        return jsonify({"error": "Nem található megoldás."}), 500

    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    route.append(manager.IndexToNode(index))  # Ez a végpont

    return jsonify({"route": route})


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 8080))
    app.run(host="0.0.0.0", port=port)
