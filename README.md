# Planificación de Trayectorias mediante Algoritmos Basados en Muestreo (RRT, RRT*, Informed RRT*, BIT*)

Este repositorio contiene una implementación modular en Python de algoritmos avanzados para la planificación de trayectorias en entornos 2D con obstáculos rectangulares. El proyecto se enfoca en la búsqueda de caminos óptimos minimizando la distancia euclidiana y garantizando la seguridad del robot mediante detección de colisiones proactiva.

## 🛠️ Estructura del Software

El código está organizado siguiendo principios de diseño modular para facilitar su mantenimiento y escalabilidad:

* **`src/structures.py`**: Contiene las clases base geométricas (`Point`, `Segment`) y las estructuras de datos para el árbol de búsqueda (`Node`, `Tree`).
* **`src/env.py`**: Gestiona el entorno de simulación, incluyendo la definición de obstáculos y las funciones de verificación de colisiones tanto para puntos como para trayectorias completas.
* **`src/rrt.py`**: Implementa la lógica de los algoritmos de planificación: RRT, RRT*, Informed RRT* y BIT* (Batch Informed Trees).
* **`notebooks/notebookCompleto.ipynb`**: Entorno interactivo para la ejecución de experimentos, configuración de escenarios y visualización de resultados.
* **`results/`**: Directorio destinado al almacenamiento de figuras, animaciones y archivos de logs generados durante las pruebas. Viene completo con todos los resultados que genera el estado base del `notebookCompleto.ipynb`.

## 🚀 Instalación

Para configurar el proyecto en su entorno local, siga estas instrucciones:

1.  **Requisitos previos**: Asegúrese de tener instalado Python 3.8 o superior.
2.  **Clonar el repositorio**:
    ```bash
    git clone https://github.com/Andro5/trabajo-11-IAR-Liborio-Roman-Andres-Rubio
    cd trabajo-11-IAR-Liborio-Roman-Andres-Rubio
    ```
3.  **Instalar dependencias**:
    ```bash
    pip install -r requirements.txt
    ```

## 🧪 Reproducción de Experimentos

Para validar el funcionamiento de los algoritmos y replicar los resultados obtenidos en la memoria:

1.  Inicie Jupyter Notebook o abra el proyecto en VS Code: `notebooks/notebookCompleto.ipynb`.
2.  Ejecute las celdas de inicialización para cargar las clases de la carpeta `src/`.
3.  Ejecute las celdas del notebook atendiendo a sus explicaciones o modifíque sus parámetros para obtener los diferentes resultados.
4.  Los parámetros como `iterations`, `batch_size` y `edge_length` pueden ajustarse directamente en el notebook para observar su impacto en la convergencia hacia la solución óptima.

## 💡 Detalles de Implementación

### Algoritmo BIT* (Batch Informed Trees)
La implementación de BIT* ha sido optimizada para garantizar la convergencia en trayectorias rectilíneas mediante el uso de operadores de coste inclusivos (`<=`). Además, se ha desacoplado el chequeo de colisiones del proceso de muestreo para evitar bloqueos computacionales en elipses de búsqueda estrechas (estableciendo además un mínimo en el tamaño del eje mejor), delegando la validación de seguridad a la fase de expansión del árbol.



### Muestreo Informado
Los algoritmos `Informed_RRT_star` y `BIT_star` utilizan muestreo elíptico focalizado en el espacio de soluciones prometedoras, definido por:
$$dist(p, start) + dist(p, goal) \le c_{best}$$
Donde $c_{best}$ es el coste del mejor camino encontrado hasta el momento.

## 📄 Licencia

Este proyecto se distribuye bajo la licencia **MIT**, lo que permite su uso, modificación y distribución libre, cumpliendo con los requisitos de código abierto del trabajo.
