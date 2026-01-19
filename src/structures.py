# src/structures.py
from __future__ import annotations
import math
import numpy as np

class Point:
    """Representa un punto en un espacio bidimensional."""
    
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance(self, p: Point) -> float:
        """Calcula la distancia euclidiana entre este punto y otro."""
        return math.sqrt((self.x - p.x)**2 + (self.y - p.y)**2)

    def steer(self, p: Point, length: float) -> Point:
        """
        Calcula un nuevo punto en la dirección de 'p' limitado por 'length'.
        Si la distancia es menor que length, retorna una copia de 'p'.
        """
        dist = self.distance(p)
        if dist <= length: return Point(p.x, p.y)
        
        # Interpolación lineal para encontrar el punto a la distancia exacta
        new_x = self.x + (p.x - self.x) * length / dist
        new_y = self.y + (p.y - self.y) * length / dist
        return Point(new_x, new_y)

    def draw(self, ax, color='c', size=2, alpha=1.0):
        """Dibuja el punto en el gráfico usando matplotlib."""
        ax.plot(self.x, self.y, 'o', color=color, markersize=size, alpha=alpha, zorder=5)

class Segment:
    """Representa un segmento de recta definido por dos puntos."""
    
    def __init__(self, p0: Point, p1: Point):
        self.p0 = p0
        self.p1 = p1
        self.length = p0.distance(p1)

    def draw(self, ax, color='c', alpha=1.0, linewidth=1):
        """Dibuja el segmento y sus vértices en el gráfico."""
        ax.plot([self.p0.x, self.p1.x], [self.p0.y, self.p1.y], 
                marker='o', color=color, markersize=3, linewidth=linewidth, alpha=alpha)

    def distance_to_point(self, p: Point) -> float:
        """
        Calcula la distancia mínima desde un punto al segmento.
        Utiliza la proyección escalar para determinar el punto más cercano en el segmento.
        """
        if self.length < 1e-9: return self.p0.distance(p)
        
        # t es el parámetro de proyección normalizado (0 a 1)
        t = max(0, min(1, ((p.x - self.p0.x) * (self.p1.x - self.p0.x) + 
                           (p.y - self.p0.y) * (self.p1.y - self.p0.y)) / (self.length**2)))
        
        closest = Point(self.p0.x + t * (self.p1.x - self.p0.x), 
                        self.p0.y + t * (self.p1.y - self.p0.y))
        return closest.distance(p)

    def distance_to_segment(self, other: Segment) -> float:
        """Calcula la distancia mínima entre dos segmentos de recta."""
        if self.intersects(other): return 0
        
        # La distancia mínima es la menor de las distancias de los extremos a los segmentos opuestos
        return min(self.distance_to_point(other.p0), self.distance_to_point(other.p1),
                   other.distance_to_point(self.p0), other.distance_to_point(self.p1))

    def intersects(self, other: Segment) -> bool:
        """
        Determina si este segmento interseca con otro usando el test de orientación CCW.
        """
        def ccw(A, B, C):
            # Comprueba si el giro A->B->C es en sentido antihorario
            return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)
        
        # Dos segmentos intersecan si los extremos de uno están en lados opuestos del otro
        return ccw(self.p0, other.p0, other.p1) != ccw(self.p1, other.p0, other.p1) and \
               ccw(self.p0, self.p1, other.p0) != ccw(self.p0, self.p1, other.p1)

class Node:
    """Estructura de nodo para algoritmos de búsqueda basados en árboles (ej. RRT/RRT*)."""
    
    def __init__(self, point: Point, parent: Node = None):
        self.point = point
        self.parent = parent
        self.children = [] # Mantiene referencias a nodos descendentes
        
        # Cálculo del coste acumulado (distancia total desde la raíz)
        if parent is None:
            self.cost = 0
        else:
            self.cost = parent.cost + point.distance(parent.point)
            # Registro automático en la jerarquía del árbol
            parent.children.append(self)

class Tree:
    """Estructura de datos que gestiona la colección de nodos en el espacio de búsqueda."""
    
    def __init__(self, x0: float, y0: float):
        """Inicializa el árbol con un nodo raíz en las coordenadas dadas."""
        self.nodes = [Node(Point(x0, y0))]

    def get_nearest_node(self, p: Point) -> Node:
        """Busca y retorna el nodo del árbol más cercano al punto dado."""
        return min(self.nodes, key=lambda n: n.point.distance(p))

    def get_near_nodes(self, p: Point, radius: float) -> list[Node]:
        """Retorna una lista de todos los nodos que se encuentran dentro de un radio determinado."""
        return [n for n in self.nodes if n.point.distance(p) <= radius]

    def get_optimal_path(self, goal: Point, threshold: float) -> list[Node] | None:
        """
        Encuentra el camino con menor coste desde la raíz hasta el objetivo.
        Busca nodos dentro del umbral (threshold) de la meta y reconstruye la trayectoria.
        """
        # Filtrar nodos que han llegado satisfactoriamente cerca del objetivo
        nodes_near_goal = [n for n in self.nodes if n.point.distance(goal) <= threshold]
        if not nodes_near_goal:
            return None
        
        # Criterio RRT*: Minimizar el coste acumulado + la distancia final estimada a la meta
        best_end_node = min(nodes_near_goal, key=lambda n: n.cost + n.point.distance(goal))
        
        # Reconstrucción del camino desde el nodo final hacia la raíz siguiendo los padres
        path = []
        curr = best_end_node
        while curr is not None:
            path.append(curr)
            curr = curr.parent
            
        return path[::-1] # Invierte la lista para que el orden sea Inicio -> Fin