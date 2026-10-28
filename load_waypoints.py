import csv


def load_waypoints(csv_path):
    """
    Read a track centerline CSV with a header row "x,y".

    Returns two plain Python lists (wp_x, wp_y), which pybind11
    converts to std::vector<double> for MpccController.
    """
    wp_x = []
    wp_y = []

    with open(csv_path, "r") as file:
        reader = csv.reader(file)
        next(reader)                          # skip the header row "x,y"

        for row in reader:                    # each row looks like ["-0.1774", "0.1463"]
            if len(row) < 2:                  # skip empty lines
                continue
            wp_x.append(float(row[0]))        # text -> number
            wp_y.append(float(row[1]))

    return wp_x, wp_y


# quick check: run "python load_waypoints.py" from the MPCC folder
if __name__ == "__main__":
    wp_x, wp_y = load_waypoints("budapest_centerline.csv")
    print("number of waypoints:", len(wp_x))
    print("first point:", wp_x[0], wp_y[0])
    print("last point: ", wp_x[-1], wp_y[-1])
