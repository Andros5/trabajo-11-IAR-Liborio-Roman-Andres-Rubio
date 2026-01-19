# src/rrt.py
from __future__ import annotations
import random
import math
import numpy as np
from typing import TYPE_CHECKING, Tuple, List, Optional
from structures import Point, Node, Tree, Segment

# Evita importaciones circulares para el chequeo de tipos
if TYPE_CHECKING:
    from env import Scenario

class RRT(Tree):
    """Algoritmo Rapidly-exploring Random Tree estándar para planificación de caminos."""

    def iterate(self, scenario: Scenario, edge_length: float, robot_radius: float) -> Tuple[Optional[Node], Point, Node]:
        """
        Realiza una iteración de expansión del árbol RRT.
        1. Muestrea un punto aleatorio.
        2. Encuentra el nodo más cercano.
        3. Avanza una distancia fija (steer).
        4. Añade el nodo si no hay colisión.
        """
        p_rand = Point(random.uniform(scenario.xmin, scenario.xmax), 
                       random.uniform(scenario.ymin, scenario.ymax))
        nearest = self.get_nearest_node(p_rand)
        p_new = nearest.point.steer(p_rand, edge_length)
        
        if not scenario.collision_with_trajectory(Segment(nearest.point, p_new), robot_radius):
            nnew = Node(p_new, nearest)
            self.nodes.append(nnew)
            return nnew, p_new, nearest
        return None, p_new, nearest

class RRT_star(Tree):
    """Extensión RRT* que optimiza el coste del camino mediante 'rewiring' (recableado)."""

    def __init__(self, x0: float, y0: float, gamma: float = 50.0):
        """Inicializa RRT* con un parámetro gamma que controla el radio de búsqueda de vecinos."""
        super().__init__(x0, y0)
        self.gamma = gamma

    def _update_children_costs(self, node: Node) -> None:
        """Propaga de forma recursiva los cambios de coste hacia todos los nodos descendientes."""
        for child in node.children:
            child.cost = node.cost + child.point.distance(node.point)
            self._update_children_costs(child)

    def iterate(self, scenario: Scenario, edge_length: float, robot_radius: float) -> Tuple[Optional[Node], Point, Node]:
        """Muestrea un punto y expande el árbol usando la lógica de optimización RRT*."""
        p_rand = Point(random.uniform(scenario.xmin, scenario.xmax), 
                       random.uniform(scenario.ymin, scenario.ymax))
        nearest = self.get_nearest_node(p_rand)
        p_new = nearest.point.steer(p_rand, edge_length)
        return self.expand_tree(p_new, nearest, scenario, edge_length, robot_radius)

    def expand_tree(self, p_new: Point, nearest_node: Node, scenario: Scenario, 
                    edge_length: float, robot_radius: float) -> Tuple[Optional[Node], Point, Node]:
        """
        Lógica central de RRT*: 
        1. Encuentra el mejor padre en un radio cercano (Choose Parent).
        2. Inserta el nodo.
        3. Reasigna padres a nodos cercanos si el nuevo nodo ofrece un camino más corto (Rewiring).
        """
        if scenario.collision_with_trajectory(Segment(nearest_node.point, p_new), robot_radius):
            return None, p_new, nearest_node

        # Cálculo del radio adaptativo para búsqueda de vecinos cercanos
        n_nodes = len(self.nodes)
        radius = min(edge_length * 2.0, self.gamma * math.sqrt(math.log(n_nodes) / n_nodes))
        near_nodes = self.get_near_nodes(p_new, radius)
        
        # --- Fase 1: Elegir el mejor padre (Choose Parent) ---
        best_n, min_c = nearest_node, nearest_node.cost + nearest_node.point.distance(p_new)
        for n in near_nodes:
            c = n.cost + n.point.distance(p_new)
            if c < min_c and not scenario.collision_with_trajectory(Segment(n.point, p_new), robot_radius):
                min_c, best_n = c, n
        
        nnew = Node(p_new, best_n)
        self.nodes.append(nnew)
        
        # --- Fase 2: Recableado (Rewiring) ---
        for n in near_nodes:
            new_potential_cost = nnew.cost + nnew.point.distance(n.point)
            if new_potential_cost < n.cost:
                if not scenario.collision_with_trajectory(Segment(nnew.point, n.point), robot_radius):
                    # Cambiar el padre del nodo cercano al nuevo nodo recién creado
                    if n.parent:
                        n.parent.children.remove(n)
                    n.parent = nnew
                    nnew.children.append(n)
                    n.cost = new_potential_cost
                    self._update_children_costs(n)
                    
        return nnew, p_new, best_n

class Informed_RRT_star(RRT_star):
    """Informed RRT* que utiliza una elipse de muestreo una vez encontrado el primer camino."""

    def __init__(self, x0: float, y0: float, x_goal: float, y_goal: float, gamma: float = 50.0):
        super().__init__(x0, y0, gamma)
        self.start, self.goal = Point(x0, y0), Point(x_goal, y_goal)
        self.c_best = float('inf') # Longitud del mejor camino encontrado hasta ahora

    def _update_global_best_cost(self, threshold: float) -> None:
        """Actualiza el coste del mejor camino buscando nodos que alcancen el objetivo."""
        nodes_near_goal = [n for n in self.nodes if n.point.distance(self.goal) <= threshold]
        if nodes_near_goal:
            best_total_cost = min(n.cost + n.point.distance(self.goal) for n in nodes_near_goal)
            self.c_best = min(self.c_best, best_total_cost)

    def iterate(self, scenario: Scenario, edge_length: float, robot_radius: float) -> Tuple[Optional[Node], Point, Node]:
        """Iteración que utiliza muestreo informado si ya existe una solución previa."""
        p_rand = self._sample(scenario)
        nearest = self.get_nearest_node(p_rand)
        p_new = nearest.point.steer(p_rand, edge_length)
        res = self.expand_tree(p_new, nearest, scenario, edge_length, robot_radius)
        
        self._update_global_best_cost(edge_length)
        return res

    def _sample(self, scenario: Scenario) -> Point:
        """Muestreo elíptico basado en el coste actual para restringir el espacio de búsqueda."""
        if self.c_best == float('inf'):
            # Muestreo uniforme si no hay camino previo
            return Point(random.uniform(scenario.xmin, scenario.xmax), 
                         random.uniform(scenario.ymin, scenario.ymax))
        
        c_min = self.start.distance(self.goal)
        center = np.array([(self.start.x + self.goal.x)/2.0, (self.start.y + self.goal.y)/2.0])
        angle = math.atan2(self.goal.y - self.start.y, self.goal.x - self.start.x)
        
        while True:
            # Radios de la elipse basados en el coste actual y la distancia mínima
            r1, r2 = self.c_best / 2.0, math.sqrt(max(0.1, self.c_best**2 - c_min**2)) / 2.0
            rho, phi = math.sqrt(random.random()), random.random() * 2.0 * math.pi
            
            # Generación de punto en elipse y rotación según la orientación start-goal
            x, y = rho * r1 * math.cos(phi), rho * r2 * math.sin(phi)
            x_rot = x * math.cos(angle) - y * math.sin(angle)
            y_rot = x * math.sin(angle) + y * math.cos(angle)
            p = Point(x_rot + center[0], y_rot + center[1])
            
            if (scenario.xmin <= p.x <= scenario.xmax and scenario.ymin <= p.y <= scenario.ymax):
                return p

class BIT_star(Informed_RRT_star):
    """Batch Informed Trees (BIT*). Procesa muestras por lotes y las ordena por potencial de mejora."""

    def __init__(self, x0: float, y0: float, x_goal: float, y_goal: float, batch_size: int = 30, gamma: float = 50.0):
        super().__init__(x0, y0, x_goal, y_goal, gamma)
        self.batch_size = batch_size
        self.samples: List[Point] = []
        self.c_current = float('inf')

    def iterate(self, scenario: Scenario, edge_length: float, robot_radius: float) -> Tuple[Optional[Node], Point, Node]:
        """
        Realiza una iteración del algoritmo BIT*. 
        Procesa muestras por lotes (batches) priorizando aquellas con mayor potencial de mejora.
        """
        # 1. Generación del Lote (Batch)
        # Si no quedan muestras, generamos un lote completo de nuevos puntos.
        if not self.samples:
            while len(self.samples) < self.batch_size:
                # El método _sample ya garantiza matemáticamente que el punto está dentro de 
                # la elipse definida por c_best. No comprobamos colisiones aquí para evitar 
                # bloqueos en escenarios muy estrechos.
                self.samples.append(self._sample(scenario))
            
            # Ordenamos las muestras por su heurística admisible del inicio a la meta pasando por ellas
            self.samples.sort(key=lambda p: p.distance(self.start) + p.distance(self.goal))
            self.c_current = self.c_best

        # 2. Selección de la mejor muestra
        # Extraemos el punto más prometedor del lote actual
        p_test = self.samples.pop(0)
        self.c_current = p_test.distance(self.start) + p_test.distance(self.goal)
        
        # 3. Conexión al árbol
        # Buscamos el nodo existente más cercano para intentar integrarlo en el árbol
        nearest = self.get_nearest_node(p_test)
        
        # Aplicamos steer para limitar el crecimiento máximo por paso. 
        # Gracias a la implementación de Point.steer, si p_test está a menos de edge_length, 
        # se usará p_test directamente.
        p_new = nearest.point.steer(p_test, edge_length)
            
        # 4. Expansión y Optimización (Rewiring)
        # delegamos en expand_tree, que verificará colisiones en la trayectoria y 
        # buscará el mejor padre posible dentro del radio de cercanía.
        res = self.expand_tree(p_new, nearest, scenario, edge_length, robot_radius)
        
        # 5. Actualización del coste global
        # Registramos si se ha encontrado una mejor conexión al objetivo tras la expansión
        old_c = self.c_best
        self._update_global_best_cost(edge_length)
        
        # 6. Poda (Pruning) dinámica
        # Si c_best ha mejorado, limpiamos el lote de muestras que ya no son competitivas.
        # Usamos <= para permitir que el árbol siga refinándose en casos de líneas rectas óptimas.
        if self.c_best < old_c:
            self.samples = [s for s in self.samples if s.distance(self.start) + s.distance(self.goal) <= self.c_best]
        
        return res