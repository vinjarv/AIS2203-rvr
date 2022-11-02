import json
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use("TKAgg")

with open("test.json") as f:
    data = json.load(f)

nodes = data["nodes"]
ids = nodes.keys()

plt.figure(1)

coords_x = []
coords_y = []
for id in ids:
    node = nodes[id]
    connected = node["connected_nodes"]
    connected = connected.replace(" ", "").split(",")
    if connected[0] != "":
        # Plot node connection lines
        for neighbour in connected:
            plt.plot([float(node["x"]), float(nodes[neighbour]["x"])], [float(node["y"]), float(nodes[neighbour]["y"])], color="orange")
    # Store coordinates of node
    coords_x.append(float(node["x"]))
    coords_y.append(float(node["y"]))

# Plot all nodes
plt.scatter(coords_x, coords_y, color="k")
# Plot text label
for i in range(len(ids)):
    plt.text(coords_x[i]+0.1, coords_y[i]+0.1, list(ids)[i], fontsize=7, color="green")

# Plot origin
plt.quiver(0, 0, 1, 0, scale_units="xy", scale=1/1, color="r")
plt.quiver(0, 0, 0, 1, scale_units="xy", scale=1/1, color="b")

plt.xlim(-2, 15)
plt.ylim(-2, 15)
plt.grid()
plt.title("Navigation nodes")
plt.show()
