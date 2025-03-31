import gtsam
from gtsam import Pose2, BetweenFactorPose2, PriorFactorPose2, noiseModel, NonlinearFactorGraph, Values, symbol
import numpy as np

def spiral_with_drift(step, offset=np.array([0, 0]), heading=0.0, a=0.9, b=0.1, drift_per_step=0.1):
    theta = 0.3 * step
    r = a + b * theta
    x_local = r * np.cos(theta)
    y_local = r * np.sin(theta)
    pos_local = np.array([x_local, y_local])

    # Apply global drift along heading direction
    drift = np.array([np.cos(heading), np.sin(heading)]) * drift_per_step * step
    pos_global = pos_local + drift + offset

    yaw = theta + heading + np.pi / 2
    return Pose2(pos_global[0], pos_global[1], yaw)

def create_spiral_robot_graph(num_steps=60, offset_steps=10, loop_threshold=0.7):
    graph = NonlinearFactorGraph()
    initial_estimate = Values()
    id_map = {}
    current_id = 0

    # Noise models
    odom_noise = noiseModel.Diagonal.Sigmas([0.05, 0.05, 0.02])
    loop_noise = noiseModel.Diagonal.Sigmas([0.02, 0.02, 0.01])
    prior_noise = noiseModel.Diagonal.Sigmas([0.01, 0.01, 0.01])

    def add_vertex(sym, pose):
        nonlocal current_id
        key = symbol(sym[0], sym[1])
        id_map[key] = current_id
        initial_estimate.insert(key, pose)
        current_id += 1

    poses_a = []
    poses_b = []

    # Robot A: spiral drifting along +x
    for i in range(num_steps):
        pa = spiral_with_drift(i, offset=np.array([0, 0]), heading=0.0)
        add_vertex(('a', i), pa)
        poses_a.append(pa)
        if i > 0:
            delta = poses_a[i - 1].between(pa)
            graph.add(BetweenFactorPose2(symbol('a', i - 1), symbol('a', i), delta, odom_noise))

    # Robot B: spiral drifting along +y, starts at [2, -2]
    for i in range(num_steps):
        if i - offset_steps < 0:
            continue
        pb = spiral_with_drift(i - offset_steps, offset=np.array([-2, 5]), heading=0.0)
        add_vertex(('b', i), pb)
        poses_b.append((i, pb))
        if len(poses_b) > 1:
            i_prev, pb_prev = poses_b[-2]
            delta = pb_prev.between(pb)
            graph.add(BetweenFactorPose2(symbol('b', i_prev), symbol('b', i), delta, odom_noise))

    # Prior on a0
    graph.add(PriorFactorPose2(symbol('a', 0), poses_a[0], prior_noise))

    # Loop closures where A[i] is close to B[i]
    for i in range(offset_steps, num_steps):
        pose_a = poses_a[i]
        pose_b = spiral_with_drift(i - offset_steps, offset=np.array([2, -2]), heading=np.pi / 2)
        dist = np.linalg.norm([pose_a.x() - pose_b.x(), pose_a.y() - pose_b.y()])
        if dist < loop_threshold:
            graph.add(BetweenFactorPose2(symbol('a', i), symbol('b', i), pose_a.between(pose_b), loop_noise))

    return graph, initial_estimate, id_map


def write_g2o(graph, estimates, filename="two_robot.g2o"):
    with open(filename, 'w') as f:
        # Vertices
        for key in estimates.keys():
            pose = estimates.atPose2(key)
            idx =key
            f.write(f"VERTEX_SE2 {idx} {pose.x()} {pose.y()} {pose.theta()}\n")

        # Edges
        for i in range(graph.size()):
            factor = graph.at(i)
            if isinstance(factor, BetweenFactorPose2):
                i1 = factor.keys()[0]
                i2 = factor.keys()[1]
                t = factor.measured()
                info = factor.noiseModel().information()
                info_vals = upper_triangle(info)
                f.write(f"EDGE_SE2 {i1} {i2} {t.x()} {t.y()} {t.theta()} {' '.join(map(str, info_vals))}\n")

def upper_triangle(info_matrix):
    indices = [(0,0), (0,1), (0,2), (1,1), (1,2), (2,2)]
    return [info_matrix[i,j] for i,j in indices]

import matplotlib.pyplot as plt

def plot_spiral_graph(initial_estimate, graph, id_map):
    fig, ax = plt.subplots(figsize=(8, 8))

    # Separate trajectories
    poses_a = []
    poses_b = []
    for key in initial_estimate.keys():
        sym = gtsam.Symbol(key)
        pose = initial_estimate.atPose2(key)
        if sym.chr() == ord('a'):
            poses_a.append((sym.index(), pose.x(), pose.y()))
        elif sym.chr() == ord('b'):
            poses_b.append((sym.index(), pose.x(), pose.y()))

    # Sort by index
    poses_a.sort()
    poses_b.sort()
    ax.plot([p[1] for p in poses_a], [p[2] for p in poses_a], 'b-', label='Robot A')
    ax.plot([p[1] for p in poses_b], [p[2] for p in poses_b], 'g-', label='Robot B')

    # Plot loop closures
    for i in range(graph.size()):
        factor = graph.at(i)
        if isinstance(factor, BetweenFactorPose2):
            key1 = factor.keys()[0]
            key2 = factor.keys()[1]
            sym1 = gtsam.Symbol(key1)
            sym2 = gtsam.Symbol(key2)

            # Inter-robot constraint: a ↔ b
            if sym1.chr() != sym2.chr():
                p1 = initial_estimate.atPose2(key1)
                p2 = initial_estimate.atPose2(key2)
                ax.plot([p1.x(), p2.x()], [p1.y(), p2.y()], 'r--', linewidth=1)

    ax.set_title("Spiral Robot Trajectories with Loop Closures")
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    plt.tight_layout()
    plt.show()

def generate_spiral_trajectories(num_steps=60, offset_steps=10):
    poses_a = []
    poses_b = []
    for i in range(num_steps):
        pa = spiral_with_drift(i, offset=np.array([0, 0]), heading=0.0)
        poses_a.append(pa)

        if i - offset_steps >= 0:
            pb = spiral_with_drift(i - offset_steps, offset=np.array([0, 3]), heading=0)
            poses_b.append((i, pb))  # index aligns with A's timeline

    return poses_a, poses_b  # poses_b: List[(index, Pose2)]

def build_graph_from_trajectories(poses_a, poses_b, odom_noise, prior_noise):
    graph = NonlinearFactorGraph()
    initial = Values()
    id_map = {}
    current_id = 0

    def add_vertex(sym, pose):
        nonlocal current_id
        key = symbol(sym[0], sym[1])
        id_map[key] = current_id
        initial.insert(key, pose)
        current_id += 1

    for i, p in enumerate(poses_a):
        add_vertex(('a', i), p)
        if i > 0:
            delta = poses_a[i - 1].between(p)
            graph.add(BetweenFactorPose2(symbol('a', i - 1), symbol('a', i), delta, odom_noise))

    for idx, p in poses_b:
        add_vertex(('b', idx), p)

    for k in range(1, len(poses_b)):
        i_prev, p_prev = poses_b[k - 1]
        i_curr, p_curr = poses_b[k]
        delta = p_prev.between(p_curr)
        graph.add(BetweenFactorPose2(symbol('b', i_prev), symbol('b', i_curr), delta, odom_noise))

    # Prior
    graph.add(PriorFactorPose2(symbol('a', 0), poses_a[0], prior_noise))

    return graph, initial, id_map

def add_loop_closures_post_process(graph, poses_a, poses_b, loop_noise, threshold=0.6):
    for i, pa in enumerate(poses_a):
        for j, pb_entry in poses_b:
            dist = np.linalg.norm([pa.x() - pb_entry.x(), pa.y() - pb_entry.y()])
            if dist < threshold:
                rel = pa.between(pb_entry)
                graph.add(BetweenFactorPose2(symbol('a', i), symbol('b', j), rel, loop_noise))

# Generate
poses_a, poses_b = generate_spiral_trajectories(num_steps=80, offset_steps=10)

# Build graph
odom_noise = gtsam.noiseModel.Diagonal.Sigmas([0.05, 0.05, 0.02])
prior_noise = gtsam.noiseModel.Diagonal.Sigmas([0.01, 0.01, 0.01])
loop_noise = gtsam.noiseModel.Diagonal.Sigmas([0.02, 0.02, 0.01])

graph, initial, id_map = build_graph_from_trajectories(poses_a, poses_b, odom_noise, prior_noise)

# Add loop closures in post-process
add_loop_closures_post_process(graph, poses_a, poses_b, loop_noise, threshold=0.6)

# Save & Plot
write_g2o(graph, initial, filename="spiral_postprocessed.g2o")
plot_spiral_graph(initial, graph, id_map)
