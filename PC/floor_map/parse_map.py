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
            x = float(node["x"]); y = float(node["y"])
            nx = float(nodes[neighbour]["x"]); ny = float(nodes[neighbour]["y"])
            dx = (nx-x); dy = (ny-y)
            l = (dx**2 + dy**2)**0.5
            dx, dy = (dx*0.3/l, dy*0.3/l)
            plt.plot([x, nx], [y, ny], color="orange")
            plt.arrow(x, y, dx, dy, width=0.05, head_width=5*0.05, color="black")
    # Store coordinates of node
    coords_x.append(float(node["x"]))
    coords_y.append(float(node["y"]))

# Plot all nodes
plt.scatter(coords_x, coords_y, color="k")
# Plot text label
for i in range(len(ids)):
    plt.text(coords_x[i]+0.2, coords_y[i]+0.2, list(ids)[i], fontsize=7, color="green")

# Plot origin
plt.quiver(0, 0, 1, 0, scale_units="xy", scale=1/1, color="r")
plt.quiver(0, 0, 0, 1, scale_units="xy", scale=1/1, color="b")

plt.xlim(-2, 15)
plt.ylim(-2, 15)
plt.grid()
plt.title("Navigation nodes")
plt.show()
