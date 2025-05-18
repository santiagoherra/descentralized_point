import numpy as np
import pdb

# Variables para pruebas:
current_x = 0
current_y = 0

def obtener_trayectoria(waypoints, current_x, current_y):
    """
    Encuentra el valor más cercano a current_x y current_y en una lista de waypoints.
    """
    # Ordenar el waypoint usando sort.
    # Separar columnas
    x_vector = waypoints[:, 0]
    y_vector = waypoints[:, 1]

    # Ordenar de forma independiente
    x_sorted = np.sort(x_vector)
    y_sorted = np.sort(y_vector)

# Crear el array resultante: 2 columnas, ya sin relación X-Y original
    sorted_waypoint = np.column_stack((x_sorted, y_sorted))

    # Obtener el punto mas cercano a la trayectoria y el que le sigue para X
    left = 0
    right = len(sorted_waypoint[:, 0]) - 1

    while left <= right:
        mid = left + (right - left) // 2

        if sorted_waypoint[mid, 0] == current_x:
            try:
                trajectory_x = sorted_waypoint[mid + 5, 0]  # offset
            except:
                trajectory_x = sorted_waypoint[0, 0]

        if current_x < sorted_waypoint[mid, 0]:
            right = mid - 1
        else:
            left = mid + 1

    if abs(sorted_waypoint[left - 1, 0] - current_x) <= abs(sorted_waypoint[right, 0] - current_x):
        try:
            trajectory_x = sorted_waypoint[left + 1, 0]
        except:
            trajectory_x = sorted_waypoint[0, 0]
    else:
        try:
            trajectory_x = sorted_waypoint[right + 1, 0]
        except:
            trajectory_x = sorted_waypoint[0, 0]

    # Repetir el algoritmo para Y
    left = 0
    right = len(sorted_waypoint[:, 1]) - 1

    while left <= right:
        mid = left + (right - left) // 2

        if sorted_waypoint[mid, 1] == current_y:
            try:
                trajectory_y = sorted_waypoint[mid + 5, 1]  # offset
            except:
                trajectory_y = sorted_waypoint[0, 1]

        if current_y < sorted_waypoint[mid, 1]:
            right = mid - 1
        else:
            left = mid + 1

    if abs(sorted_waypoint[left - 1, 1] - current_y) <= abs(sorted_waypoint[right, 1] - current_y):
        try:
            trajectory_y = sorted_waypoint[left + 1, 1]
        except:
            trajectory_y = sorted_waypoint[0, 1]
    else:
        try:
            trajectory_y = sorted_waypoint[right + 1, 1]
        except:
            trajectory_y = sorted_waypoint[0, 1]

    return trajectory_x, trajectory_y

# Generar waypoints aleatorios (10x2)

waypoints = np.array([
    [1.0,  2.0],
    [3.5,  5.0],
    [2.2,  8.1],
    [7.5,  1.5],
    [6.3,  4.8],
    [9.0,  0.5],
    [4.2,  7.3],
    [5.5,  3.2],
    [8.4,  6.6],
    [0.9,  9.0]
])

while True:
    # Ejecutar prueba
    current_x = float(input("da el valor de x\n"))
    current_y = float(input("da el valor de y\n"))


    trajectory_x, trajectory_y = obtener_trayectoria(waypoints, current_x, current_y)

    print(f"Resultado -> trajectory_x: {trajectory_x}, trajectory_y: {trajectory_y}")






def obtener_trayectoria(self, waypoints):
        """
        Encuentra el valor más cercano a 'target' en una lista ordenada 'arr'.
        :param arr: Lista ordenada de números.
        :param target: Valor objetivo.
        :return: El valor más cercano.
        """
        
        # Ordenar el waypoint usando sort.
        # Separar columnas
        x_vector = waypoints[:, 0]
        y_vector = waypoints[:, 1]

        # Ordenar de forma independiente
        x_sorted = np.sort(x_vector)
        y_sorted = np.sort(y_vector)

        # Crear el array resultante: 2 columnas, ya sin relación X-Y original
        sorted_waypoint = np.column_stack((x_sorted, y_sorted))

        # Obtener el punto mas cercano a la trayectoria y el que le sigue
        # para la coordenada x
        left = 0
        right = len(sorted_waypoint[:, 0]) - 1

        # Búsqueda binaria modificada.
        while left <= right:
            mid = left + (right - left) // 2

            if sorted_waypoint[mid, 0] == self.current_x:
                try:
                    self.trajectory_x = sorted_waypoint[mid + 20, 0] # offset que se puede cambiar
                except:
                    self.trajectory_x = sorted_waypoint[0, 0]

            # Decidir si buscamos a la izquierda o derecha.
            if self.current_x < sorted_waypoint[mid, 0]:
                right = mid - 1
            else:
                left = mid + 1

        # Al final del while:
        # right es el menor índice con arr[right] <= target.
        # left es el mayor índice con arr[left] >= target.
        # Ahora comparamos quién está más cerca. Si la distancia es igual para los dos
        # se agarra el mayor valor.
        if abs(sorted_waypoint[left - 1, 0] - self.current_x) <= abs(sorted_waypoint[right, 0] - self.current_x):
            # Cambia el valor de la trayectoria, es posible que tenga que cambiar el offset de la trajectoria
            try: # Si se sale del indice vuelve al inicio de la trajectoria
                self.trajectory_x = sorted_waypoint[left + 20, 0]
            except:
                self.trajectory_x = sorted_waypoint[0, 0]
        else:
            try:
                self.trajectory_x = sorted_waypoint[right + 20, 0]
            except:
                self.trajectory_x = sorted_waypoint[0, 0]

        # Ahora se realiza el mismo algoritmo para encontrar el menor valor en la 
        # coordenada Y

        left = 0
        right = len(sorted_waypoint[:, 1]) - 1

        while left <= right:
            mid = left + (right - left) // 2

            if sorted_waypoint[mid, 1] == self.current_y:
                try:
                    self.trajectory_y = sorted_waypoint[mid + 20, 1] # offset que se puede cambiar
                except:
                    self.trajectory_y = sorted_waypoint[0, 1]

            if self.current_y < sorted_waypoint[mid, 1]:
                right = mid - 1
            else:
                left = mid + 1

        if abs(sorted_waypoint[left - 1, 1] - self.current_y) <= abs(sorted_waypoint[right, 1] - self.current_y):
            try:
                self.trajectory_y = sorted_waypoint[left + 20, 1]
            except:
                self.trajectory_y = sorted_waypoint[0, 1]
        else:
            try:
                self.trajectory_y = sorted_waypoint[right + 20, 1]
            except:
                self.trajectory_y = sorted_waypoint[0, 1]
        return None
