# src/env.py
from __future__ import annotations
from matplotlib import pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from matplotlib.axes import Axes
from structures import Point, Segment

class Scenario:
    """Gestiona el entorno de simulación, incluyendo los límites y los obstáculos."""
    
    def __init__(self, obstacles: list[Obstacle], xmin: float = -30, xmax: float = 30, 
                 ymin: float = -30, ymax: float = 30, plot: bool = False):
        """
        Inicializa el escenario con una lista de obstáculos y límites espaciales.
        Si plot es True, configura la figura de matplotlib.
        """
        self.obstacles = obstacles
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        
        if plot:
            self.fig, self.ax = plt.subplots()
            self.ax.set_aspect('equal', adjustable='box')
            self.ax.set_xlim((xmin, xmax))
            self.ax.set_ylim((ymin, ymax))
            self.ax.grid()

            for obstacle in self.obstacles:
                obstacle.draw(self.ax)

    def collision_with_robot(self, point: Point, radius: float) -> bool:
        """
        Verifica si un robot (representado por un punto y un radio) 
        colisiona con cualquier obstáculo del escenario.
        """
        return any(o.collision_with_point(point, radius) for o in self.obstacles)

    def collision_with_trajectory(self, segment: Segment, radius: float) -> bool:
        """
        Verifica si una trayectoria (segmento) con un grosor dado (radius)
        interseca con algún obstáculo.
        """
        return any(o.collision_with_trajectory(segment, radius) for o in self.obstacles)

class Obstacle:
    """Representa un obstáculo rectangular en el espacio 2D."""
    
    def __init__(self, x: float, y: float, width: float, height: float):
        """
        Define un obstáculo desde su esquina inferior izquierda y sus dimensiones.
        Calcula automáticamente los puntos de las esquinas y los segmentos de los lados.
        """
        self.x, self.y = x, y
        self.width, self.height = width, height
        self.p0 = Point(x, y)
        self.p1 = Point(x + width, y)
        self.p2 = Point(x + width, y + height)
        self.p3 = Point(x, y + height)
        
        # Define el perímetro como una lista de segmentos
        self.sides = [
            Segment(self.p0, self.p1), Segment(self.p1, self.p2), 
            Segment(self.p2, self.p3), Segment(self.p3, self.p0)
        ]

    def draw(self, ax: Axes):
        """Dibuja el obstáculo como un rectángulo azul en el eje proporcionado."""
        ax.add_patch(Rectangle((self.x, self.y), self.width, self.height, 
                               color='b'))

    def collision_with_point(self, p: Point, radius: float) -> bool:
        """
        Comprueba si un punto (con un margen de radio) colisiona con el obstáculo.
        Retorna True si el punto está dentro o si su distancia a los lados es menor al radio.
        """
        # Comprobación de si el punto está estrictamente dentro del rectángulo
        inside = (self.x <= p.x <= self.x + self.width) and (self.y <= p.y <= self.y + self.height)
        if inside: return True
        
        # Comprobación de proximidad a los bordes
        return any(s.distance_to_point(p) <= radius for s in self.sides)

    def collision_with_trajectory(self, trajectory: Segment, radius: float) -> bool:
        """
        Comprueba si una trayectoria rectilínea interseca con el obstáculo.
        Considera tanto los extremos del segmento como la intersección de líneas.
        """
        # Si alguno de los extremos ya está en colisión, hay colisión total
        if self.collision_with_point(trajectory.p0, radius) or \
           self.collision_with_point(trajectory.p1, radius): return True
        
        # Comprueba si el segmento de trayectoria cruza alguno de los lados del obstáculo
        return any(s.distance_to_segment(trajectory) <= radius for s in self.sides)