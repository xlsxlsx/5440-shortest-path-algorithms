import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import math
from bidirectional_algo import Graph, BiDijkstraStepper

NODE_R = 25  # Increased for better visibility

# Default node positions for the example graph - maximum spread
DEFAULT_POSITIONS = {
    'T': (450, 60),      # Top center
    'G': (700, 150),     # Top right
    'D': (550, 220),     # Mid right
    'F': (250, 220),     # Mid left
    'A': (100, 150),     # Top left
    's': (50, 380),      # Bottom left
    'E': (450, 380),     # Bottom center
    'C': (300, 550),     # Bottom left-center
    'B': (150, 550),     # Bottom far left
}


class GraphLayoutEngine:
    """Force-directed graph layout algorithm."""
    
    def __init__(self, graph, width=750, height=550):
        self.graph = graph
        self.width = width
        self.height = height
        self.positions = {}
    
    def calculate_layout(self, iterations=400):
        """Calculate node positions using force-directed layout."""
        nodes = list(self.graph.nodes)
        
        # Initialize positions randomly with maximum spread
        import random
        random.seed(42)  # For reproducibility
        for node in nodes:
            self.positions[node] = (
                random.uniform(200, self.width - 200),
                random.uniform(200, self.height - 200)
            )
        
        # Force-directed layout with extremely strong repulsion
        k = math.sqrt((self.width * self.height) / len(nodes)) * 6.0  # Maximum spacing
        temperature = self.width / 2  # Very high initial temperature
        
        for iteration in range(iterations):
            # Calculate repulsive forces between all pairs
            forces = {node: [0.0, 0.0] for node in nodes}
            
            for node1 in nodes:
                for node2 in nodes:
                    if node1 == node2:
                        continue
                    
                    x1, y1 = self.positions[node1]
                    x2, y2 = self.positions[node2]
                    dx, dy = x1 - x2, y1 - y2
                    distance = math.sqrt(dx*dx + dy*dy) + 0.01
                    
                    # Repulsive force
                    force = k * k / distance
                    fx, fy = (dx / distance) * force, (dy / distance) * force
                    
                    forces[node1][0] += fx
                    forces[node1][1] += fy
                    forces[node2][0] -= fx
                    forces[node2][1] -= fy
            
            # Attraction along edges (minimal strength)
            for node in self.graph.forward_graph:
                for neighbor, _ in self.graph.forward_graph[node]:
                    x1, y1 = self.positions[node]
                    x2, y2 = self.positions[neighbor]
                    dx, dy = x2 - x1, y2 - y1
                    distance = math.sqrt(dx*dx + dy*dy) + 0.01
                    
                    # Attractive force (reduced by factor of 5)
                    force = (distance * distance / k) / 5.0
                    fx, fy = (dx / distance) * force, (dy / distance) * force
                    
                    forces[node][0] += fx
                    forces[node][1] += fy
                    forces[neighbor][0] -= fx
                    forces[neighbor][1] -= fy
            
            # Update positions
            for node in nodes:
                fx, fy = forces[node]
                displacement = math.sqrt(fx*fx + fy*fy) + 0.01
                
                # Limit displacement by temperature
                if displacement > temperature:
                    fx = (fx / displacement) * temperature
                    fy = (fy / displacement) * temperature
                
                x, y = self.positions[node]
                x += fx
                y += fy
                
                # Keep within bounds with larger margin
                margin = 80
                x = max(margin, min(self.width - margin, x))
                y = max(margin, min(self.height - margin, y))
                
                self.positions[node] = (x, y)
            
            # Cool down (slower cooling for better convergence)
            temperature *= 0.97
        
        return self.positions


class GraphConfigDialog:
    def __init__(self, parent):
        self.parent = parent
        self.result = None

        self.dialog = tk.Toplevel(parent)
        self.dialog.title("Graph Configuration")
        self.dialog.geometry("600x500")
        self.dialog.transient(parent)
        self.dialog.grab_set()

        # Center the dialog
        self.dialog.geometry("+%d+%d" % (
            parent.winfo_rootx() + 50,
            parent.winfo_rooty() + 50
        ))

        self.create_widgets()

    def create_widgets(self):
        main_frame = ttk.Frame(self.dialog)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Graph type selection
        type_frame = ttk.LabelFrame(main_frame, text="Graph Type")
        type_frame.pack(fill=tk.X, pady=(0, 10))

        self.graph_type = tk.StringVar(value="undirected")
        ttk.Radiobutton(type_frame, text="Undirected Graph",
                        variable=self.graph_type, value="undirected").pack(anchor=tk.W, pady=5)
        ttk.Radiobutton(type_frame, text="Directed Graph",
                        variable=self.graph_type, value="directed").pack(anchor=tk.W, pady=5)

        # Source and target nodes
        nodes_frame = ttk.LabelFrame(main_frame, text="Source and Target Nodes")
        nodes_frame.pack(fill=tk.X, pady=(0, 10))

        nodes_subframe = ttk.Frame(nodes_frame)
        nodes_subframe.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(nodes_subframe, text="Source:").grid(row=0, column=0, padx=(0, 10), pady=5)
        self.source_var = tk.StringVar(value="s")
        ttk.Entry(nodes_subframe, textvariable=self.source_var, width=10).grid(row=0, column=1, pady=5)

        ttk.Label(nodes_subframe, text="Target:").grid(row=0, column=2, padx=(20, 10), pady=5)
        self.target_var = tk.StringVar(value="T")
        ttk.Entry(nodes_subframe, textvariable=self.target_var, width=10).grid(row=0, column=3, pady=5)

        # Graph data input
        data_frame = ttk.LabelFrame(main_frame, text="Graph Data Input")
        data_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        ttk.Label(data_frame, text="Enter edges in format: Node1-Node2: Weight").pack(anchor=tk.W, padx=5, pady=(5, 0))
        ttk.Label(data_frame, text="Example: T-G: 3").pack(anchor=tk.W, padx=5)

        self.data_text = scrolledtext.ScrolledText(data_frame, height=15, width=50)
        self.data_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Load default example
        self.load_default_example()

        # Buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X)

        ttk.Button(button_frame, text="Load Default Example",
                   command=self.load_default_example).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(button_frame, text="Clear",
                   command=self.clear_data).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(button_frame, text="Cancel",
                   command=self.dialog.destroy).pack(side=tk.RIGHT, padx=(10, 0))
        ttk.Button(button_frame, text="OK",
                   command=self.on_ok).pack(side=tk.RIGHT)

    def load_default_example(self):
        default_data = """T-G: 3
T-F: 9
T-D: 13
G-E: 5
F-E: 2
F-C: 6
E-C: 4
E-B: 3
E-D: 12
D-A: 9
A-s: 4
s-B: 5
s-C: 7"""
        self.data_text.delete('1.0', tk.END)
        self.data_text.insert('1.0', default_data)

    def clear_data(self):
        self.data_text.delete('1.0', tk.END)

    def on_ok(self):
        source = self.source_var.get().strip()
        target = self.target_var.get().strip()
        graph_type = self.graph_type.get()
        data = self.data_text.get('1.0', tk.END).strip()

        if not source or not target:
            messagebox.showerror("Error", "Please specify both source and target nodes.")
            return

        if not data:
            messagebox.showerror("Error", "Please enter graph data.")
            return

        # Parse the graph data
        edges = []
        lines = data.split('\n')
        for line in lines:
            line = line.strip()
            if not line:
                continue

            # Try to parse the line
            if ':' in line:
                parts = line.split(':')
                if len(parts) != 2:
                    messagebox.showerror("Error", f"Invalid format: {line}")
                    return

                nodes_part = parts[0].strip()
                weight_str = parts[1].strip()

                # Parse nodes
                if '-' in nodes_part:
                    node_parts = nodes_part.split('-')
                    if len(node_parts) != 2:
                        messagebox.showerror("Error", f"Invalid node format: {nodes_part}")
                        return
                    node1, node2 = node_parts[0].strip(), node_parts[1].strip()
                else:
                    messagebox.showerror("Error", f"Invalid node format: {nodes_part}")
                    return

                # Parse weight
                try:
                    weight = int(weight_str)
                except ValueError:
                    messagebox.showerror("Error", f"Invalid weight: {weight_str}")
                    return

                edges.append((node1, node2, weight))
            else:
                messagebox.showerror("Error", f"Invalid format: {line}")
                return

        self.result = {
            'source': source,
            'target': target,
            'graph_type': graph_type,
            'edges': edges
        }
        self.dialog.destroy()


class VizApp:
    def __init__(self, root, graph: Graph, source, target, graph_type='undirected'):
        self.root = root
        self.graph = graph
        self.source = source
        self.target = target
        self.graph_type = graph_type  # 'directed' or 'undirected'

        root.title(f"Bidirectional Dijkstra Visualizer - {graph_type.upper()} Graph")
        root.geometry("1500x900")  # Very large window for maximum spacing

        # Modern color scheme
        self.colors = {
            'source': '#10B981',
            'target': '#EF4444',
            'forward_active': '#3B82F6',
            'backward_active': '#F59E0B',
            'forward_final': '#1E40AF',
            'backward_final': '#D97706',
            'relax_edge': '#FBBF24',
            'meeting': '#8B5CF6',
            'final_path': '#10B981',
            'candidate': '#A78BFA',
            'canvas_bg': '#F9FAFB',
            'text_bg': '#FFFFFF',
            'node_default': '#E5E7EB',
            'node_border': '#6B7280',
            'edge_default': '#9CA3AF',
        }

        # Create main frame
        main_frame = ttk.Frame(root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Create left canvas area
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Create right info area
        right_frame = ttk.Frame(main_frame, width=400)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        right_frame.pack_propagate(False)

        # Create control panel FIRST - pack at bottom to ensure it's always visible
        control_frame = ttk.Frame(left_frame)
        control_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(5, 5))

        # Canvas - pack AFTER controls so it fills remaining space
        self.canvas = tk.Canvas(left_frame, bg=self.colors['canvas_bg'],
                                highlightthickness=1, highlightbackground="#ccc")
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=(5, 5), pady=(5, 0))

        # Step information display
        self.step_info = tk.StringVar()
        self.step_info.set("Click 'Next Step' to start the algorithm")
        step_label = ttk.Label(control_frame, textvariable=self.step_info, font=('Arial', 10, 'bold'))
        step_label.pack(side=tk.LEFT, padx=(0, 10))

        # Buttons
        self.btn_next = ttk.Button(control_frame, text="Next Step (Space)", command=self.on_next)
        self.btn_next.pack(side=tk.LEFT, padx=5)

        self.btn_reset = ttk.Button(control_frame, text="Reset (R)", command=self.on_reset)
        self.btn_reset.pack(side=tk.LEFT, padx=5)

        # Speed control
        speed_frame = ttk.Frame(control_frame)
        speed_frame.pack(side=tk.LEFT, padx=20)

        ttk.Label(speed_frame, text="Speed:").pack(side=tk.LEFT)
        self.speed_var = tk.DoubleVar(value=1.0)
        speed_scale = ttk.Scale(speed_frame, from_=0.1, to=2.0, variable=self.speed_var,
                                orient=tk.HORIZONTAL, length=100)
        speed_scale.pack(side=tk.LEFT, padx=5)

        # Auto-play button
        self.auto_playing = False
        self.btn_auto = ttk.Button(control_frame, text="Auto Play (A)", command=self.toggle_auto_play)
        self.btn_auto.pack(side=tk.LEFT, padx=5)

        # Reconfigure button
        self.btn_reconfig = ttk.Button(control_frame, text="Reconfigure Graph", command=self.reconfigure)
        self.btn_reconfig.pack(side=tk.LEFT, padx=5)

        # Status information area
        notebook = ttk.Notebook(right_frame)
        notebook.pack(fill=tk.BOTH, expand=True)

        # Algorithm status tab
        status_frame = ttk.Frame(notebook)
        notebook.add(status_frame, text="Algorithm Status")

        self.status = scrolledtext.ScrolledText(status_frame, width=45, height=28, wrap='word',
                                                bg=self.colors['text_bg'])
        self.status.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Graph information tab
        graph_frame = ttk.Frame(notebook)
        notebook.add(graph_frame, text="Graph Info")

        graph_info = scrolledtext.ScrolledText(graph_frame, width=45, height=28, wrap='word', bg=self.colors['text_bg'])
        graph_info.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Fill graph information
        graph_info.insert(tk.END, "Graph Structure:\n\n")
        for node, neighbors in graph.forward_graph.items():
            graph_info.insert(tk.END, f"{node}: {neighbors}\n")
        graph_info.config(state=tk.DISABLED)

        # Legend tab
        legend_frame = ttk.Frame(notebook)
        notebook.add(legend_frame, text="Legend")

        legend_text = scrolledtext.ScrolledText(legend_frame, width=45, height=28, wrap='word',
                                                bg=self.colors['text_bg'])
        legend_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        legend_info = """
Color Explanation:
- Green: Source node
- Red: Target node
- Blue: Forward search active node
- Orange: Backward search active node
- Dark Blue: Forward search finalized node
- Dark Orange: Backward search finalized node
- Yellow: Edge being relaxed
- Purple: Candidate node / Meeting node
- Bright Green: Final path

Operation Instructions:
- Space: Next step
- R: Reset
- A: Auto play/pause
        """
        legend_text.insert(tk.END, legend_info)
        legend_text.config(state=tk.DISABLED)

        # Bind keyboard events
        root.bind('<space>', lambda e: self.on_next())
        root.bind('<Key-r>', lambda e: self.on_reset())
        root.bind('<Key-a>', lambda e: self.toggle_auto_play())
        root.focus_set()

        # Calculate positions
        self.positions = self._calculate_positions()
        self.node_ids = {}
        self.edge_ids = {}
        self.edge_weight_ids = {}
        self.node_labels = {}

        self.stepper = BiDijkstraStepper(graph, source, target)
        self.gen = self.stepper.step()

        self.draw_graph()
        self.last_snap = None
        self.auto_id = None

        # Initial step
        self.on_next()

    def _calculate_positions(self):
        """Calculate node positions."""
        all_nodes = self.graph.nodes
        if all_nodes and all(node in DEFAULT_POSITIONS for node in all_nodes):
            return DEFAULT_POSITIONS.copy()
        
        # Use very large canvas size for maximum spacing
        layout_engine = GraphLayoutEngine(self.graph, width=950, height=700)
        positions = layout_engine.calculate_layout()
        return positions
    
    def _draw_arrow_head(self, x1, y1, x2, y2, color='#9CA3AF'):
        """Manually draw an arrow head at the end of a line."""
        # Calculate arrow direction
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        if length < 0.1:
            return
        
        # Normalize direction
        dx /= length
        dy /= length
        
        # Arrow parameters - larger for better visibility
        arrow_length = 18
        arrow_width = 10
        
        # Shorten the arrow to not overlap with node
        end_x = x2 - dx * (NODE_R + 2)
        end_y = y2 - dy * (NODE_R + 2)
        
        # Calculate arrow head points
        base_x = end_x - dx * arrow_length
        base_y = end_y - dy * arrow_length
        
        # Perpendicular vector
        perp_x = -dy * arrow_width
        perp_y = dx * arrow_width
        
        # Arrow triangle points
        points = [
            end_x, end_y,
            base_x + perp_x, base_y + perp_y,
            base_x - perp_x, base_y - perp_y
        ]
        
        return self.canvas.create_polygon(points, fill=color, outline=color)

    def draw_graph(self):
        """Draw the graph with arrows for directed graphs."""
        self.canvas.delete('all')
        drawn = set()
        
        # Draw edges
        for u in self.graph.forward_graph:
            for v, w in self.graph.forward_graph[u]:
                # For undirected graphs, skip reverse edges
                if self.graph_type == 'undirected' and (v, u) in drawn:
                    continue
                
                x1, y1 = self.positions[u]
                x2, y2 = self.positions[v]
                
                # Draw edge
                edge_width = 3 if self.graph_type == 'directed' else 2
                line = self.canvas.create_line(
                    x1, y1, x2, y2, 
                    width=edge_width, 
                    fill=self.colors['edge_default'],
                    capstyle=tk.ROUND
                )
                self.edge_ids[(u, v)] = line
                
                # Draw arrow head manually for directed graphs
                if self.graph_type == 'directed':
                    arrow_id = self._draw_arrow_head(x1, y1, x2, y2, self.colors['edge_default'])
                    self.edge_ids[(u, v, 'arrow')] = arrow_id
                
                drawn.add((u, v))
        
        # Draw weight labels
        for key in list(self.edge_ids.keys()):
            # Skip arrow entries
            if len(key) == 3:
                continue
            
            u, v = key
            weight = None
            for neighbor, w in self.graph.forward_graph.get(u, []):
                if neighbor == v:
                    weight = w
                    break
            
            if weight is not None:
                x1, y1 = self.positions[u]
                x2, y2 = self.positions[v]
                mx, my = (x1 + x2) / 2, (y1 + y2) / 2
                
                bg_oval = self.canvas.create_oval(
                    mx - 22, my - 14, mx + 22, my + 14,  # Larger background
                    fill='white',
                    outline='#D1D5DB',
                    width=1
                )
                text_id = self.canvas.create_text(
                    mx, my, 
                    text=str(weight), 
                    font=('Arial', 10, 'bold'),  # Larger font
                    fill='#1F2937'
                )
                self.edge_weight_ids[(u, v)] = (bg_oval, text_id)
        
        # Draw nodes
        for n, (x, y) in self.positions.items():
            oval = self.canvas.create_oval(
                x - NODE_R, y - NODE_R, x + NODE_R, y + NODE_R,
                fill='white', 
                outline=self.colors['node_border'], 
                width=2
            )
            
            txt = self.canvas.create_text(
                x, y, 
                text=str(n), 
                font=('Arial', 13, 'bold'),  # Larger font
                fill='#1F2937'
            )
            
            dist_label = self.canvas.create_text(
                x, y + NODE_R + 15,
                text='',
                font=('Arial', 9),  # Larger font
                fill='#6B7280'
            )
            
            self.node_ids[n] = (oval, txt)
            self.node_labels[n] = dist_label

    def apply_snapshot(self, snap):
        """Apply snapshot with arrow preservation."""
        self.last_snap = snap
        action = snap.get('action', '')
        self.step_info.set(f"Step: {action}")

        # Reset node colors
        for n, (oval, txt) in self.node_ids.items():
            base = 'white'
            border = self.colors['node_border']
            border_width = 2
            
            if n == self.source:
                base = self.colors['source']
                border = '#059669'
                border_width = 2.5
            if n == self.target:
                base = self.colors['target']
                border = '#DC2626'
                border_width = 2.5
            
            self.canvas.itemconfig(oval, fill=base, outline=border, width=border_width)
        
        # Reset edge colors
        for key, eid in self.edge_ids.items():
            edge_width = 3 if self.graph_type == 'directed' else 2
            if len(key) == 3:  # Arrow head
                self.canvas.itemconfig(eid, fill=self.colors['edge_default'], outline=self.colors['edge_default'])
            else:  # Edge line
                self.canvas.itemconfig(eid, fill=self.colors['edge_default'], width=edge_width)
        
        # Update distance labels
        dist_start = snap.get('distance_start', {})
        dist_goal = snap.get('distance_goal', {})
        for n in self.node_ids:
            label_parts = []
            if n in dist_start:
                label_parts.append(f"s:{dist_start[n]:.0f}")
            if n in dist_goal:
                label_parts.append(f"t:{dist_goal[n]:.0f}")
            label_text = ' | '.join(label_parts) if label_parts else ''
            self.canvas.itemconfig(self.node_labels[n], text=label_text)

        # Apply highlights
        h = snap.get('highlight', {})
        
        if 'pop' in h:
            side, node = h['pop']
            color = self.colors['forward_active'] if side == 'forward' else self.colors['backward_active']
            border_color = '#1E40AF' if side == 'forward' else '#D97706'
            self.canvas.itemconfig(self.node_ids[node][0], fill=color, outline=border_color, width=4)
        
        if 'finalize' in h:
            side, node = h['finalize']
            color = self.colors['forward_final'] if side == 'forward' else self.colors['backward_final']
            border_color = '#1E3A8A' if side == 'forward' else '#92400E'
            self.canvas.itemconfig(self.node_ids[node][0], fill=color, outline=border_color, width=3.5)
        
        if 'consider_relax' in h:
            u, v = h['consider_relax']
            self.canvas.itemconfig(self.node_ids[u][0], outline='#10B981', width=4)
            self.canvas.itemconfig(self.node_ids[v][0], outline='#10B981', width=4)
        
        if 'relax' in h:
            u, v = h['relax']
            edge_id = self.edge_ids.get((u, v)) or self.edge_ids.get((v, u))
            if edge_id:
                self.canvas.itemconfig(edge_id, fill=self.colors['relax_edge'], width=4)
                # Update arrow color too
                arrow_id = self.edge_ids.get((u, v, 'arrow')) or self.edge_ids.get((v, u, 'arrow'))
                if arrow_id:
                    self.canvas.itemconfig(arrow_id, fill=self.colors['relax_edge'], outline=self.colors['relax_edge'])
            self.canvas.itemconfig(self.node_ids[u][0], fill='#86EFAC', outline='#10B981', width=3)
            self.canvas.itemconfig(self.node_ids[v][0], fill='#BBF7D0', outline='#10B981', width=3)
        
        if 'candidate' in h:
            node, cost = h['candidate']
            self.canvas.itemconfig(self.node_ids[node][0], fill=self.colors['candidate'], outline='#7C3AED', width=3.5)
        
        if 'meeting' in h:
            node = h['meeting']
            self.canvas.itemconfig(self.node_ids[node][0], fill=self.colors['meeting'], outline='#6D28D9', width=5)
            x, y = self.positions[node]
            self.canvas.create_oval(
                x - NODE_R - 5, y - NODE_R - 5,
                x + NODE_R + 5, y + NODE_R + 5,
                outline='#8B5CF6', width=2, dash=(5, 3), tags='temp_highlight'
            )
        
        if 'final_path' in h and h['final_path']:
            path = h['final_path']
            for i in range(len(path) - 1):
                u, v = path[i], path[i + 1]
                edge_id = self.edge_ids.get((u, v)) or self.edge_ids.get((v, u))
                if edge_id:
                    self.canvas.itemconfig(edge_id, fill=self.colors['final_path'], width=5)
                    # Update arrow color too
                    arrow_id = self.edge_ids.get((u, v, 'arrow')) or self.edge_ids.get((v, u, 'arrow'))
                    if arrow_id:
                        self.canvas.itemconfig(arrow_id, fill=self.colors['final_path'], outline=self.colors['final_path'])
            for n in path:
                self.canvas.itemconfig(self.node_ids[n][0], fill=self.colors['final_path'], outline='#059669', width=4)

        # Update best path information
        best = snap.get('best', float('inf'))
        meet = snap.get('meet_node', None)
        label = f"Current shortest path: {best if best != float('inf') else '∞'}"
        if meet:
            label += f"   Meeting node: {meet}"
        self.canvas.delete('best_label')
        self.canvas.create_text(20, 20, text=label, anchor='w', font=('Arial', 12, 'bold'),
                                fill='#333', tag='best_label')

        # Update status text
        self.status.config(state=tk.NORMAL)
        self.status.delete('1.0', tk.END)
        self.status.insert(tk.END, f"Action: {action}\n\n")

        self.status.insert(tk.END, "Start queue (min first):\n")
        for item in sorted(snap.get('queue_start', [])):
            self.status.insert(tk.END, f"  {item}\n")

        self.status.insert(tk.END, "\nGoal queue (min first):\n")
        for item in sorted(snap.get('queue_goal', [])):
            self.status.insert(tk.END, f"  {item}\n")

        self.status.insert(tk.END, "\nStart distances:\n")
        for k, v in sorted(snap.get('distance_start', {}).items(), key=lambda x: str(x[0])):
            self.status.insert(tk.END, f"  {k}: {v}\n")

        self.status.insert(tk.END, "\nGoal distances:\n")
        for k, v in sorted(snap.get('distance_goal', {}).items(), key=lambda x: str(x[0])):
            self.status.insert(tk.END, f"  {k}: {v}\n")

        self.status.insert(tk.END, "\nStart visited nodes:\n")
        self.status.insert(tk.END, f"  {sorted(list(snap.get('visited_start', [])))}\n")

        self.status.insert(tk.END, "\nGoal visited nodes:\n")
        self.status.insert(tk.END, f"  {sorted(list(snap.get('visited_goal', [])))}\n")

        self.status.config(state=tk.DISABLED)
        self.status.see(tk.END)

    def on_next(self):
        try:
            snap = next(self.gen)
            self.apply_snapshot(snap)
        except StopIteration:
            self.status.config(state=tk.NORMAL)
            self.status.insert(tk.END, "\nAlgorithm completed!\n")
            self.status.config(state=tk.DISABLED)
            self.step_info.set("Algorithm completed!")
            if self.auto_playing:
                self.toggle_auto_play()

    def on_reset(self):
        if self.auto_playing:
            self.toggle_auto_play()

        self.stepper = BiDijkstraStepper(self.graph, self.source, self.target)
        self.gen = self.stepper.step()
        self.draw_graph()
        self.step_info.set("Click 'Next Step' to start algorithm")
        self.on_next()

    def reconfigure(self):
        config_dialog = GraphConfigDialog(self.root)
        self.root.wait_window(config_dialog.dialog)

        if config_dialog.result:
            config = config_dialog.result
            self.source = config['source']
            self.target = config['target']

            self.graph = Graph()
            self.graph_type = config['graph_type']
            for u, v, w in config['edges']:
                if config['graph_type'] == 'undirected':
                    self.graph.add_undirected_edge(u, v, w)
                else:
                    self.graph.add_edge(u, v, w)

            self.update_positions()
            self.on_reset()
            self.update_graph_info()

    def update_positions(self):
        self.positions = self._calculate_positions()

    def update_graph_info(self):
        for child in self.root.winfo_children():
            if isinstance(child, ttk.Frame):
                for subchild in child.winfo_children():
                    if isinstance(subchild, ttk.Frame):
                        for notebook_child in subchild.winfo_children():
                            if isinstance(notebook_child, ttk.Notebook):
                                graph_frame = notebook_child.nametowidget(notebook_child.tabs()[1])
                                for widget in graph_frame.winfo_children():
                                    if isinstance(widget, scrolledtext.ScrolledText):
                                        widget.config(state=tk.NORMAL)
                                        widget.delete('1.0', tk.END)
                                        widget.insert(tk.END, "Graph Structure:\n\n")
                                        for node, neighbors in self.graph.forward_graph.items():
                                            widget.insert(tk.END, f"{node}: {neighbors}\n")
                                        widget.config(state=tk.DISABLED)
                                        return

    def toggle_auto_play(self):
        if self.auto_playing:
            self.auto_playing = False
            self.btn_auto.config(text="Auto Play (A)")
            if self.auto_id:
                self.root.after_cancel(self.auto_id)
                self.auto_id = None
        else:
            self.auto_playing = True
            self.btn_auto.config(text="Pause (A)")
            self.auto_play_step()

    def auto_play_step(self):
        if self.auto_playing:
            self.on_next()
            delay = int(1000 / self.speed_var.get())
            self.auto_id = self.root.after(delay, self.auto_play_step)


def build_default_graph():
    g = Graph()
    edges = [
        ('T', 'G', 3), ('T', 'F', 9), ('T', 'D', 13),
        ('G', 'E', 5), ('F', 'E', 2), ('F', 'C', 6),
        ('E', 'C', 4), ('E', 'B', 3), ('E', 'D', 12),
        ('D', 'A', 9), ('A', 's', 4), ('s', 'B', 5), ('s', 'C', 7)
    ]
    for u, v, w in edges:
        g.add_undirected_edge(u, v, w)
    return g


def main():
    root = tk.Tk()

    config_dialog = GraphConfigDialog(root)
    root.wait_window(config_dialog.dialog)

    if config_dialog.result:
        config = config_dialog.result
        source = config['source']
        target = config['target']

        g = Graph()
        graph_type = config['graph_type']
        for u, v, w in config['edges']:
            if graph_type == 'undirected':
                g.add_undirected_edge(u, v, w)
            else:
                g.add_edge(u, v, w)

        app = VizApp(root, g, source, target, graph_type)
        root.mainloop()
    else:
        root.destroy()


if __name__ == "__main__":
    main()
