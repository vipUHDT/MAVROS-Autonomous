import os, json
def load_search_area_waypoints():
        """
        Load waypoints from JSON file
        """
        # Change this path if needed
        wp_path = "waypoints.json"
        if not os.path.exists(wp_path):
            return

        with open(wp_path, 'r') as f:
            search_area_waypoints = json.load(f)["search_waypoints"]
        return search_area_waypoints


if __name__ == "__main__":
     print(load_search_area_waypoints())