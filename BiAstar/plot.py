import numpy as np
import matplotlib.pyplot as plt
from heapq import heappush, heappop
import math
import time
import tracemalloc
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class GraphNode:
    def __init__(self, node_id, x, y):
        self.node_id = node_id
        self.x = x
        self.y = y
        self.neighbors = {}  # neighbor_id -> weight
        self.forward_dist = float('inf')
        self.backward_dist = float('inf')
        self.forward_processed = False
        self.backward_processed = False
        self.forward_prev = None
        self.backward_prev = None


class BidirectionalAStar:
    def __init__(self, nodes, heuristic_type='landmark', landmarks=None):
        self.nodes = nodes
        self.forward_heap = []
        self.backward_heap = []
        self.forward_visited = set()  # 记录前向搜索访问的节点
        self.backward_visited = set()  # 记录后向搜索访问的节点
        self.steps = []  # 记录每一步的状态
        self.heuristic_type = heuristic_type
        self._min_edge_weight = None
        self.landmarks = landmarks or []
        self._lm_dists = {}

    def heuristic(self, node1_id, node2_id):
        if self.heuristic_type == 'zero':
            return 0.0
        if self.heuristic_type == 'landmark':
            if not self._lm_dists:
                return 0.0
            h_vals = []
            for L in self.landmarks:
                d1 = self._lm_dists[L].get(node1_id, float('inf'))
                d2 = self._lm_dists[L].get(node2_id, float('inf'))
                if d1 == float('inf') or d2 == float('inf'):
                    continue
                h_vals.append(abs(d1 - d2))
            return max(h_vals) if h_vals else 0.0
        return 0.0

    def _dijkstra_from(self, start_id):
        dist = {nid: float('inf') for nid in self.nodes}
        dist[start_id] = 0.0
        heap = [(0.0, start_id)]
        while heap:
            g, u = heappop(heap)
            if g > dist[u]:
                continue
            for v, w in self.nodes[u].neighbors.items():
                ng = g + w
                if ng < dist[v]:
                    dist[v] = ng
                    heappush(heap, (ng, v))
        return dist

    def _prepare_landmarks(self, source_id, target_id):
        if not self.landmarks:
            picks = []
            if source_id in self.nodes:
                picks.append(source_id)
            if target_id in self.nodes and target_id not in picks:
                picks.append(target_id)
            center = max(self.nodes.values(), key=lambda n: len(n.neighbors)).node_id
            if center not in picks:
                picks.append(center)
            self.landmarks = picks
        self._lm_dists = {L: self._dijkstra_from(L) for L in self.landmarks}

    def reset_distances(self):
        """重置所有距离"""
        for node in self.nodes.values():
            node.forward_dist = float('inf')
            node.backward_dist = float('inf')
            node.forward_processed = False
            node.backward_processed = False
            node.forward_prev = None
            node.backward_prev = None
        self.forward_visited = set()
        self.backward_visited = set()
        self.steps = []
        self.forward_heap = []
        self.backward_heap = []

    def relax_forward_edges(self, current_id):
        """松弛前向边"""
        current = self.nodes[current_id]
        self.forward_visited.add(current_id)
        for neighbor_id, weight in current.neighbors.items():
            neighbor = self.nodes[neighbor_id]
            if not neighbor.forward_processed:
                new_dist = current.forward_dist + weight
                if new_dist < neighbor.forward_dist:
                    neighbor.forward_dist = new_dist
                    neighbor.forward_prev = current_id
                    heappush(self.forward_heap, (new_dist + self.heuristic(neighbor_id, self.target_id), neighbor_id))

    def relax_backward_edges(self, current_id):
        """松弛后向边"""
        current = self.nodes[current_id]
        self.backward_visited.add(current_id)
        for neighbor_id, weight in current.neighbors.items():
            neighbor = self.nodes[neighbor_id]
            if not neighbor.backward_processed:
                new_dist = current.backward_dist + weight
                if new_dist < neighbor.backward_dist:
                    neighbor.backward_dist = new_dist
                    neighbor.backward_prev = current_id
                    heappush(self.backward_heap, (new_dist + self.heuristic(neighbor_id, self.source_id), neighbor_id))

    def get_intersecting_node(self):
        """找到交汇节点"""
        best_distance = float('inf')
        meeting_node = None

        # 检查前向处理的节点
        for node_id, node in self.nodes.items():
            if node.forward_processed and node.backward_dist != float('inf'):
                total_dist = node.forward_dist + node.backward_dist
                if total_dist < best_distance:
                    best_distance = total_dist
                    meeting_node = node_id

        # 检查后向处理的节点
        for node_id, node in self.nodes.items():
            if node.backward_processed and node.forward_dist != float('inf'):
                total_dist = node.forward_dist + node.backward_dist
                if total_dist < best_distance:
                    best_distance = total_dist
                    meeting_node = node_id

        return meeting_node, best_distance if best_distance != float('inf') else -1

    def compute_best_bridge(self):
        best_total = float('inf')
        best_u = None
        best_v = None
        for u_id, u in self.nodes.items():
            if not u.forward_processed:
                continue
            for v_id, w in u.neighbors.items():
                v = self.nodes[v_id]
                if v.backward_dist != float('inf'):
                    total = u.forward_dist + w + v.backward_dist
                    if total < best_total:
                        best_total = total
                        best_u = u_id
                        best_v = v_id
        if best_total == float('inf'):
            return None, None, -1
        return best_u, best_v, best_total

    def reconstruct_path(self, meeting_node):
        """重构路径"""
        # 前向路径
        forward_path = []
        current = meeting_node
        while current is not None:
            forward_path.append(current)
            current = self.nodes[current].forward_prev
        forward_path.reverse()

        # 后向路径
        backward_path = []
        current = self.nodes[meeting_node].backward_prev
        while current is not None:
            backward_path.append(current)
            current = self.nodes[current].backward_prev

        # 合并路径
        return forward_path + backward_path

    def reconstruct_path_via_edge(self, u_id, v_id):
        forward_path = []
        current = u_id
        while current is not None:
            forward_path.append(current)
            current = self.nodes[current].forward_prev
        forward_path.reverse()
        backward_path = []
        current = v_id
        while current is not None:
            backward_path.append(current)
            current = self.nodes[current].backward_prev
        return forward_path + backward_path

    def find_shortest_path(self, source_id, target_id):
        """查找最短路径"""
        self.source_id = source_id
        self.target_id = target_id
        self.reset_distances()
        if self.heuristic_type == 'landmark':
            self._prepare_landmarks(source_id, target_id)

        # 初始化
        self.nodes[source_id].forward_dist = 0
        self.nodes[target_id].backward_dist = 0
        heappush(self.forward_heap, (0, source_id))
        heappush(self.backward_heap, (0, target_id))

        step = 0
        best_distance = float('inf')
        meeting_node = None
        meeting_edge = None
        while self.forward_heap or self.backward_heap:
            did = False
            if self.forward_heap:
                min_f_fwd, nid_fwd = heappop(self.forward_heap)
                if not self.nodes[nid_fwd].forward_processed:
                    self.nodes[nid_fwd].forward_processed = True
                    self.relax_forward_edges(nid_fwd)
                    did = True
                    step += 1
                    cand_node, cand_dist = self.get_intersecting_node()
                    bu, bv, bdist = self.compute_best_bridge()
                    if bdist != -1 and bdist < best_distance:
                        best_distance = bdist
                        meeting_node = None
                        meeting_edge = (bu, bv)
                    if cand_dist != -1 and cand_dist < best_distance:
                        best_distance = cand_dist
                        meeting_node = cand_node
                        meeting_edge = None
                    self.steps.append({
                        'step': step,
                        'action': 'forward',
                        'forward_visited': set(self.forward_visited),
                        'backward_visited': set(self.backward_visited),
                        'forward_current': nid_fwd,
                        'backward_current': None,
                        'forward_heap': list(sorted(self.forward_heap)),
                        'backward_heap': list(sorted(self.backward_heap)),
                        'forward_distances': {nid: self.nodes[nid].forward_dist for nid in self.nodes if self.nodes[nid].forward_dist != float('inf')},
                        'backward_distances': {nid: self.nodes[nid].backward_dist for nid in self.nodes if self.nodes[nid].backward_dist != float('inf')},
                        'best_distance': best_distance if best_distance != float('inf') else None,
                        'meeting_node': meeting_node,
                        'meeting_edge': meeting_edge,
                        'best_path': self.reconstruct_path(meeting_node) if meeting_node is not None else (
                            self.reconstruct_path_via_edge(meeting_edge[0], meeting_edge[1]) if meeting_edge is not None else []
                        )
                    })

            if self.backward_heap:
                min_f_bwd, nid_bwd = heappop(self.backward_heap)
                if not self.nodes[nid_bwd].backward_processed:
                    self.nodes[nid_bwd].backward_processed = True
                    self.relax_backward_edges(nid_bwd)
                    did = True
                    step += 1
                    cand_node, cand_dist = self.get_intersecting_node()
                    bu, bv, bdist = self.compute_best_bridge()
                    if bdist != -1 and bdist < best_distance:
                        best_distance = bdist
                        meeting_node = None
                        meeting_edge = (bu, bv)
                    if cand_dist != -1 and cand_dist < best_distance:
                        best_distance = cand_dist
                        meeting_node = cand_node
                        meeting_edge = None
                    self.steps.append({
                        'step': step,
                        'action': 'backward',
                        'forward_visited': set(self.forward_visited),
                        'backward_visited': set(self.backward_visited),
                        'forward_current': None,
                        'backward_current': nid_bwd,
                        'forward_heap': list(sorted(self.forward_heap)),
                        'backward_heap': list(sorted(self.backward_heap)),
                        'forward_distances': {nid: self.nodes[nid].forward_dist for nid in self.nodes if self.nodes[nid].forward_dist != float('inf')},
                        'backward_distances': {nid: self.nodes[nid].backward_dist for nid in self.nodes if self.nodes[nid].backward_dist != float('inf')},
                        'best_distance': best_distance if best_distance != float('inf') else None,
                        'meeting_node': meeting_node,
                        'meeting_edge': meeting_edge,
                        'best_path': self.reconstruct_path(meeting_node) if meeting_node is not None else (
                            self.reconstruct_path_via_edge(meeting_edge[0], meeting_edge[1]) if meeting_edge is not None else []
                        )
                    })

            min_fwd = self.forward_heap[0][0] if self.forward_heap else float('inf')
            min_bwd = self.backward_heap[0][0] if self.backward_heap else float('inf')
            if best_distance != float('inf') and best_distance <= min_fwd and best_distance <= min_bwd:
                if meeting_node is not None:
                    path = self.reconstruct_path(meeting_node)
                elif meeting_edge is not None:
                    path = self.reconstruct_path_via_edge(meeting_edge[0], meeting_edge[1])
                else:
                    path = []
                return best_distance, path, self.forward_visited, self.backward_visited
            if not did:
                break
        return -1, [], self.forward_visited, self.backward_visited


def create_complex_graph():
    """创建一个复杂的图"""
    nodes = {}

    # 创建节点 (id, x, y)
    node_positions = [
        (0, 1, 1), (1, 3, 1), (2, 5, 1), (3, 7, 1),
        (4, 2, 3), (5, 4, 3), (6, 6, 3),
        (7, 1, 5), (8, 3, 5), (9, 5, 5), (10, 7, 5),
        (11, 2, 7), (12, 4, 7), (13, 6, 7),
        (14, 3, 9), (15, 5, 9)
    ]

    for node_id, x, y in node_positions:
        nodes[node_id] = GraphNode(node_id, x, y)

    # 添加边 (from, to, weight)
    edges = [
        (0, 1, 2), (1, 2, 3), (2, 3, 2),
        (0, 4, 3), (1, 4, 1), (1, 5, 4), (2, 5, 2), (2, 6, 3), (3, 6, 1),
        (4, 7, 2), (4, 8, 3), (5, 8, 1), (5, 9, 2), (6, 9, 3), (6, 10, 2),
        (7, 11, 1), (8, 11, 2), (8, 12, 3), (9, 12, 1), (9, 13, 2), (10, 13, 3),
        (11, 14, 2), (12, 14, 1), (12, 15, 3), (13, 15, 2),
        (14, 15, 1)
    ]

    # 添加双向边
    for from_id, to_id, weight in edges:
        nodes[from_id].neighbors[to_id] = weight
        nodes[to_id].neighbors[from_id] = weight

    return nodes


def visualize_step_by_step(nodes, algorithm, source, target, path):
    """逐步可视化算法过程"""
    plt.ion()  # 开启交互模式
    fig, ax = plt.subplots(figsize=(12, 10))

    for i, step in enumerate(algorithm.steps):
        ax.clear()

        # 绘制节点
        x_coords = [node.x for node in nodes.values()]
        y_coords = [node.y for node in nodes.values()]
        node_ids = [node.node_id for node in nodes.values()]

        ax.scatter(x_coords, y_coords, s=300, c='lightblue', edgecolors='black', alpha=0.7)

        # 标注节点ID
        for j, node_id in enumerate(node_ids):
            ax.annotate(str(node_id), (x_coords[j], y_coords[j]),
                        ha='center', va='center', fontsize=12, fontweight='bold')

        # 绘制边
        for node in nodes.values():
            for neighbor_id, weight in node.neighbors.items():
                neighbor = nodes[neighbor_id]
                ax.plot([node.x, neighbor.x], [node.y, neighbor.y],
                        'gray', alpha=0.6, linewidth=1)
                # 标注边权重
                mid_x = (node.x + neighbor.x) / 2
                mid_y = (node.y + neighbor.y) / 2
                ax.annotate(str(weight), (mid_x, mid_y),
                            ha='center', va='center', fontsize=8,
                            bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7))

        # 绘制前向搜索访问的节点（蓝色）
        if step['forward_visited']:
            forward_x = [nodes[node_id].x for node_id in step['forward_visited']]
            forward_y = [nodes[node_id].y for node_id in step['forward_visited']]
            ax.scatter(forward_x, forward_y, s=200, c='blue', edgecolors='black', alpha=0.5, marker='o')

        # 绘制后向搜索访问的节点（红色）
        if step['backward_visited']:
            backward_x = [nodes[node_id].x for node_id in step['backward_visited']]
            backward_y = [nodes[node_id].y for node_id in step['backward_visited']]
            ax.scatter(backward_x, backward_y, s=200, c='red', edgecolors='black', alpha=0.5, marker='s')

        # 高亮当前处理的节点
        if step['forward_current'] is not None:
            current_node = nodes[step['forward_current']]
            ax.scatter(current_node.x, current_node.y, s=400, c='cyan', edgecolors='black', alpha=0.9)
            ax.annotate('F', (current_node.x, current_node.y),
                        ha='center', va='center', fontsize=14, fontweight='bold', color='white')

        if step['backward_current'] is not None:
            current_node = nodes[step['backward_current']]
            ax.scatter(current_node.x, current_node.y, s=400, c='orange', edgecolors='black', alpha=0.9)
            ax.annotate('B', (current_node.x, current_node.y),
                        ha='center', va='center', fontsize=14, fontweight='bold', color='white')

        # 标记源点和目标点
        source_node = nodes[source]
        ax.scatter(source_node.x, source_node.y, s=400, c='green', edgecolors='black', alpha=0.9)
        ax.annotate('S', (source_node.x, source_node.y),
                    ha='center', va='center', fontsize=14, fontweight='bold', color='white')

        target_node = nodes[target]
        ax.scatter(target_node.x, target_node.y, s=400, c='red', edgecolors='black', alpha=0.9)
        ax.annotate('T', (target_node.x, target_node.y),
                    ha='center', va='center', fontsize=14, fontweight='bold', color='white')

        # 绘制最终路径（如果已经找到）
        if i == len(algorithm.steps) - 1 and path:
            for j in range(len(path) - 1):
                node1 = nodes[path[j]]
                node2 = nodes[path[j + 1]]
                ax.plot([node1.x, node2.x], [node1.y, node2.y],
                        'green', linewidth=4, alpha=0.8)

        ax.set_title(
            f'双向A*算法 - 第 {step["step"]} 步\n蓝色: 前向搜索, 红色: 后向搜索, 青色: 前向当前节点, 橙色: 后向当前节点')
        ax.set_xlabel('X 坐标')
        ax.set_ylabel('Y 坐标')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

        # 添加图例
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label='前向搜索'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='red', markersize=10, label='后向搜索'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='cyan', markersize=10, label='前向当前节点'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='orange', markersize=10, label='后向当前节点'),
            Line2D([0], [0], color='green', lw=4, label='最终路径')
        ]
        ax.legend(handles=legend_elements, loc='upper right')

        plt.tight_layout()
        plt.draw()
        plt.pause(1)  # 暂停1秒，让用户观察

    plt.ioff()  # 关闭交互模式
    plt.show()


def setup_chinese_font():
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']
    plt.rcParams['axes.unicode_minus'] = False
def create_grid_graph(width, height, obstacle_ratio=0.15, seed=42):
    if seed is not None:
        np.random.seed(seed)
    nodes = {}
    obstacles = set()
    total = width * height
    obstacle_count = int(total * obstacle_ratio)
    while len(obstacles) < obstacle_count:
        i = np.random.randint(0, height)
        j = np.random.randint(0, width)
        obstacles.add((i, j))
    for i in range(height):
        for j in range(width):
            if (i, j) in obstacles:
                continue
            nid = i * width + j
            nodes[nid] = GraphNode(nid, j, i)
    dirs = [(1,0),(-1,0),(0,1),(0,-1)]
    for i in range(height):
        for j in range(width):
            if (i, j) in obstacles:
                continue
            nid = i * width + j
            if nid not in nodes:
                continue
            for di, dj in dirs:
                ni, nj = i + di, j + dj
                if 0 <= ni < height and 0 <= nj < width and (ni, nj) not in obstacles:
                    nnid = ni * width + nj
                    nodes[nid].neighbors[nnid] = 1
    return nodes

def run_case(nodes, source, target, heuristic_type='euclidean'):
    alg = BidirectionalAStar(nodes, heuristic_type)
    tracemalloc.start()
    t0 = time.perf_counter()
    dist, path, fset, bset = alg.find_shortest_path(source, target)
    t1 = time.perf_counter()
    peak = tracemalloc.get_traced_memory()[1]
    tracemalloc.stop()
    return {
        'distance': dist,
        'path': path,
        'time_ms': (t1 - t0) * 1000.0,
        'expanded_forward': len(fset),
        'expanded_backward': len(bset),
        'expanded_total': len(fset) + len(bset),
        'steps': len(alg.steps),
        'peak_kb': peak / 1024.0
    }

def run_experiments():
    print("\n实验: 复杂图不同启发函数")
    nodes_c = create_complex_graph()
    for h in ['zero', 'euclidean', 'scaled_euclidean']:
        m = run_case(nodes_c, 0, 15, h)
        print(f"启发:{h} 距离:{m['distance']} 扩展:{m['expanded_total']} 时间:{m['time_ms']:.2f}ms 峰值内存:{m['peak_kb']:.1f}KB")
    print("\n实验: 网格图不同启发函数")
    nodes_g = create_grid_graph(20, 20, obstacle_ratio=0.15, seed=7)
    src = 0
    tgt = 20*20 - 1
    for h in ['zero', 'manhattan', 'euclidean']:
        m = run_case(nodes_g, src, tgt, h)
        print(f"启发:{h} 距离:{m['distance']} 扩展:{m['expanded_total']} 时间:{m['time_ms']:.2f}ms 峰值内存:{m['peak_kb']:.1f}KB")

def build_s_to_t_graph():
    coords = {
        's': (0.0, -0.5),
        'A': (2.0, 1.8),
        'B': (1.2, -2.0),
        'C': (3, 1),
        'E': (7, 0.8),
        'D': (6.2, 2.0),
        'F': (6.4, -2.0),
        'G': (8,4),
        'T': (10.0, 3.2)
    }
    nodes = {nid: GraphNode(nid, x, y) for nid, (x, y) in coords.items()}
    edges = [
        ('T', 'G', 3), ('T', 'F', 9), ('T', 'D', 13),
        ('G', 'E', 5),
        ('F', 'E', 2), ('F', 'C', 6),
        ('E', 'C', 4), ('E', 'B', 3), ('E', 'D', 12),
        ('D', 'A', 9),
        ('A', 's', 4),
        ('s', 'B', 5), ('s', 'C', 7)
    ]
    for a, b, w in edges:
        nodes[a].neighbors[b] = w
        nodes[b].neighbors[a] = w
    return nodes

class TkAStarViewer:
    def __init__(self, nodes, algorithm, source, target, path):
        self.nodes = nodes
        self.algorithm = algorithm
        self.source = source
        self.target = target
        self.path = path
        self.index = 0
        self.root = tk.Tk()
        self.root.title("双向A*演示：s 到 t")
        main = tk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True)
        left = tk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        right = tk.Frame(main, width=320)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        self.fig, self.ax = plt.subplots(figsize=(10, 7))
        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.text = tk.Text(right, font=("Consolas", 10), width=42)
        self.text.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=6)
        btn_frame = tk.Frame(right)
        btn_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=8)
        self.prev_btn = tk.Button(btn_frame, text="上一步", command=self.prev_step, width=10)
        self.prev_btn.pack(side=tk.LEFT, padx=4)
        self.next_btn = tk.Button(btn_frame, text="下一步", command=self.next_step, width=10)
        self.next_btn.pack(side=tk.LEFT, padx=4)
        self.render_step(0)

    def render_step(self, i):
        self.ax.clear()
        steps = self.algorithm.steps
        total = len(steps)
        i = max(0, min(i, total - 1))
        step = steps[i]
        x_coords = [node.x for node in self.nodes.values()]
        y_coords = [node.y for node in self.nodes.values()]
        node_ids = [node.node_id for node in self.nodes.values()]
        self.ax.scatter(x_coords, y_coords, s=360, c='#e6e6e6', edgecolors='black', alpha=1.0)
        for j, node_id in enumerate(node_ids):
            self.ax.annotate(str(node_id), (x_coords[j], y_coords[j]), ha='center', va='center', fontsize=12)
        for node in self.nodes.values():
            for neighbor_id, weight in node.neighbors.items():
                neighbor = self.nodes[neighbor_id]
                self.ax.plot([node.x, neighbor.x], [node.y, neighbor.y], color='#9e9e9e', alpha=0.8, linewidth=2)
                mid_x = (node.x + neighbor.x) / 2
                mid_y = (node.y + neighbor.y) / 2
                self.ax.annotate(str(weight), (mid_x, mid_y), ha='center', va='center', fontsize=11,
                                 bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.9))
        path_nodes = set(step.get('best_path', []))
        if path_nodes:
            px = [self.nodes[n].x for n in path_nodes]
            py = [self.nodes[n].y for n in path_nodes]
            self.ax.scatter(px, py, s=380, c='#5cb85c', edgecolors='black', alpha=1.0)
        if step['forward_current'] is not None:
            n = self.nodes[step['forward_current']]
            self.ax.scatter(n.x, n.y, s=420, c='cyan', edgecolors='black', alpha=0.9)
            self.ax.annotate('F', (n.x, n.y), ha='center', va='center', fontsize=14, color='white')
        if step['backward_current'] is not None:
            n = self.nodes[step['backward_current']]
            self.ax.scatter(n.x, n.y, s=420, c='orange', edgecolors='black', alpha=0.9)
            self.ax.annotate('B', (n.x, n.y), ha='center', va='center', fontsize=14, color='white')
        s_node = self.nodes[self.source]
        t_node = self.nodes[self.target]
        self.ax.scatter(s_node.x, s_node.y, s=420, c='green', edgecolors='black', alpha=0.9)
        self.ax.annotate('S', (s_node.x, s_node.y), ha='center', va='center', fontsize=14, color='white')
        self.ax.scatter(t_node.x, t_node.y, s=420, c='red', edgecolors='black', alpha=0.9)
        self.ax.annotate('T', (t_node.x, t_node.y), ha='center', va='center', fontsize=14, color='white')
        best_path = step.get('best_path', [])
        for j in range(len(best_path) - 1):
            n1 = self.nodes[best_path[j]]
            n2 = self.nodes[best_path[j + 1]]
            self.ax.plot([n1.x, n2.x], [n1.y, n2.y], color='#3c8f3c', linewidth=5, alpha=0.95)
        self.ax.set_title(f'双向A*算法 - 第 {step["step"]} 步')
        self.ax.set_xlabel('X 坐标')
        self.ax.set_ylabel('Y 坐标')
        self.ax.grid(True, alpha=0.3)
        self.ax.axis('equal')
        meeting = step.get('meeting_node', None)
        best_cost = step.get('best_distance', None)
        title = f"当前最短路径: {best_cost if best_cost is not None else '未发现'}    相遇节点: {meeting if meeting is not None else '无'}"
        self.ax.set_title(title)
        self.canvas.draw()
        fcur = step['forward_current'] if step['forward_current'] is not None else '无'
        bcur = step['backward_current'] if step['backward_current'] is not None else '无'
        qf = step.get('forward_heap', [])
        qb = step.get('backward_heap', [])
        fdist = step.get('forward_distances', {})
        bdist = step.get('backward_distances', {})
        lines = []
        lines.append(f"Action: {'前向' if step.get('action')=='forward' else '后向'}")
        lines.append("Start queue (min first):")
        lines.append(str(qf))
        lines.append("Goal queue (min first):")
        lines.append(str(qb))
        lines.append("Start distances:")
        lines.append("  " + ", ".join([f"{nid}: {int(d)}" for nid, d in fdist.items()]))
        lines.append("Goal distances:")
        lines.append("  " + ", ".join([f"{nid}: {int(d)}" for nid, d in bdist.items()]))
        lines.append("Start visited nodes:")
        lines.append("  " + str(sorted(list(step['forward_visited']))))
        lines.append("Goal visited nodes:")
        lines.append("  " + str(sorted(list(step['backward_visited']))))
        lines.append("")
        self.text.delete(1.0, tk.END)
        self.text.insert(tk.END, "\n".join(lines))

    def prev_step(self):
        self.index = max(0, self.index - 1)
        self.render_step(self.index)

    def next_step(self):
        self.index = min(len(self.algorithm.steps) - 1, self.index + 1)
        self.render_step(self.index)

    def run(self):
        self.root.mainloop()

setup_chinese_font()
nodes = build_s_to_t_graph()
algorithm = BidirectionalAStar(nodes, heuristic_type='landmark')
source, target = 's', 'T'
distance, path, forward_visited, backward_visited = algorithm.find_shortest_path(source, target)
TkAStarViewer(nodes, algorithm, source, target, path).run()
