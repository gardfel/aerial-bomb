import math as kk
import matplotlib.pyplot as plt
import numpy as np
import Tabl


class LA (object):

    def __init__(self, *args):
        self.y = args[0]
        self.x = args[1]
        self.v = args[2]
        self.m = args[3]
        self.p = args[4]

    def next_point(self, *args):
        self.x = self.x + 1
        self.y = self.y - 1
        self.v = self.v - 1
        v_ = 1 / self.m


def sist_ur(*args):
    La = args[0]
    v = La.m * (P * kk.cos(alf) * kk.cos(betta) - X - G * sin(omeg)) / dt
    d_omeg = ()



m_0 = 1570  # масса бомбы, кг
g = 9.80665  # ускорение свободного падения, м/с
H_0 = 6000  #

fx1400 = LA(H_0, 0, 0.8 * Tabl.tab_atm(H_0, 2), m_0, 0)

print(fx1400.v)


