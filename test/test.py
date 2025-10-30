#! /usr/bin/python3

# Librerias generales
import pytest
import descentralized_point_simulation
from descentralized_point_simulation import DescentralizedPoint, obtener_delta, definir_parametros


class Prueba_externa():
    def __init__(self):
        self.delta = obtener_delta(descentralized_point_simulation.WAYPOINTS_FILE)

    def test_obtener_delta(self):
        assert isinstance(self.delta, (int, float))
        assert self.delta != 0
        

    def test_parametros(self):
        definir_parametros(self.delta)
        # Pruebas de que las variables globales si existen
        assert hasattr(descentralized_point_simulation, "DISTANCIA_ALTA")
        assert hasattr(descentralized_point_simulation, "OFFSET_ALTO")

        # Pruebas para saber si las variables son enteros
        assert isinstance(descentralized_point_simulation. DISTANCIA_ALTA, (int, float))
        assert isinstance(descentralized_point_simulation. OFFSET_ALTO, int)
        assert isinstance(descentralized_point_simulation. OFFSET_MEDIO, int)
        assert isinstance(descentralized_point_simulation. OFFSET_BAJO, int)


    def Descentralized_point():
        return DescentralizedPoint()

        