import math as kk
import matplotlib.pyplot as plt
import numpy as np
import random
import Tabl
import Aero
import time
from mpl_toolkits.mplot3d import Axes3D
from tqdm import *
import cProfile
from distributed import Client, LocalCluster


class Target(object):

    def __init__(self, *args):
        self.v = args[2]  # random.uniform(200, 200)  # сумарная скорость цели
        self.ang = 0
        self.x0 = args[0]  # random.uniform(6000, 6000)
        self.y0 = args[1]  # random.uniform(4100, 4100)
        self.alf0 = 0  # -0.0925  # random.uniform(0, 0.3925)
        self.x = self.x0
        self.y = self.y0
        self.alf = self.alf0

    def next_coord(self, *args):

        flag_t = args[0]
        # self.alf += random.uniform(-0.0086, +0.0086)
        if flag_t <= 2:
            self.x += self.v * kk.cos(self.alf) * dt
            self.y += self.v * kk.sin(self.alf) * dt
        else:
            # self.v += 10 # * dt
            # self.alf = -0.0925 + 0.1 * (flag_t - 2)
            self.x += self.v * kk.cos(self.alf) * dt
            self.y += self.v * kk.sin(self.alf) * dt


class Rocket(object):

    def __init__(self, *args):  # 0 - коорд у цели, 1 - коорд х цели

        self.d = 0.5
        self.m = 495
        # self.w0 = 0
        # self.w1 = 0
        self.m0 = self.m  # - self.w1 - self.w0
        self.x = 0
        self.y = 12000
        self.v = 270
        self.v_ = 0
        # self.v1 = 51 + 78.9 * dt
        self.alf = 0
        self.alf_potr = 0
        self.delt = 0
        self.delt_potr = 0
        self.omega = 0  # kk.atan((args[0] - self.y) / (args[1] - self.x))  # начальное совмещение по углу
        self.d_omega = 0
        self.ips = - 0.26
        self.fi_viz = kk.atan((args[0] - self.y) / (args[1] - self.x))

        self.d_fi_viz = 0
        self.wz = 0
        self.p = 0  # 1500
        self.p1 = 0  # 1500  # сила тяги на первом режиме
        # self.p2 = 950  # 1275  # сила тяги на втором режиме
        # self.p3 = 0
        self.x_ct = 1.87
        self.q = 0

        # par_kr = args[2]
        # par_op = args[3]
        # par_korp = args[4]
        # self.paramu = args[5]

        # крылья:
        self.L_k_kr = 2.69 - self.d   # par_kr[0]
        self.S_k_kr = 0.739776  # par_kr[1]
        self.b_kr = 0.310  # par_kr[2]
        self.tan_1_kr = 0.466  # par_kr[3]
        self.L_hv_kr = 2.222  # par_kr[4]  # Расстояние от конца бортовой хорды крыла до кормового среза
        print(self.L_hv_kr)

        # рули:
        self.L_k_op = 0.5  # par_op[0]
        self.S_k_op = 0.13168  # par_op[1]
        self.b_op = 0.257  # par_op[2]
        self.tan_0_op = 0.51  # par_op[3]
        self.L1_op = 3.82  # par_op[4]  # Расстояние от носика корпуса до серидины бортовой хорды оперения
        self.x_wr_op = 3.766  # Расстояние от носика корпуса до оси вращения руля

        # нососвая часть:
        self.L_nos = 0.68  # par_korp[0]
        # кормовая часть:
        self.d_korm = 0.168  # par_korp[1]
        self.L_korm = 1.625  # par_korp[2]


    def navigation(self, *args):
        l_tra = 0
        fin_ch = 1

        h = 6.3 * 10 ** -6  # Примерная высота бугоров на поверхности корпуса (в зависимости от класса чистоты) (для 7-го)

        # параметры для метода наведения:

        a_m = 4.  # коэффициент быстроты реакции ракеты на маневр цели (от 1 до бесконечности)
        eps_krit = 10 * kk.pi / 180  # предельное отклонение координатора головки самонаведения
        # d_omega_max = 10 * kk.pi / 180
        delta_max = 25 * kk.pi / 180  # предельный угол отклонения рулей
        d_delta_max = 55 * kk.pi / 180  # * 20 * 4  # скорость отклонения рулей

        # t_st = 2.7  # время стартового участка
        # t_m = 8.3  # время маршевого участка
        # dm1 = (11.292 - 8.996) / t_st  # производная изменения массы на стартовом участке
        # dm2 = (8.996 - 7.3) / t_m  # производная изменения массы на маршевом участке
        # dx_cm1 = (0.671 - 0.78) / t_st  # скорость изменения цм на стартовом участке
        # dx_cm2 = (0.637 - 0.671) / t_m  # скорость изменеия цт на маршевом учестке

        L_f = 4.1  # длина корпуса
        L__f = L_f / L_f  # относительная длина корпуса

        l_con = 565 / 72  # относительное удлинение конической части НЧ без сферического затупления
        l_con_max = 1305 / 72  # максимальное относ. удлинение конческой части НЧ без сферического затупления
        l_zat = 23.49 / 62.59  # относительное удлинение затупления носовой части
        r_ = 2 * 0.0329 / 0.072

        Ff = 4.08  # площадь обтекаемой потоком поверхности корпуса (без донного среза)
        f_t = 0.3474925  # площадь поверхности до источника турбулизации

        # вычисление формы крыльев:
        c_kr = 0.03  # относительная толщина профиля крыла
        l_k_kr = self.L_k_kr ** 2 / self.S_k_kr  # относительное удлинение консолей крыльев
        nu_k_kr = ((self.S_k_kr / (self.L_k_kr * self.b_kr) - 0.5) ** (-1) / 2)  # относительное сужение крыльев
        b_1_kr = self.b_kr / nu_k_kr  # концевая хорда крыльев
        b_a_k_kr = 4 / 3 * self.S_k_kr / self.L_k_kr * (1 - (nu_k_kr / (nu_k_kr + 1) ** 2))  # САХ
        b_0_kr = self.b_kr * (1 + (nu_k_kr - 1) / nu_k_kr * self.d_korm / self.L_k_kr)  # корневая хорда крыльев
        S_kr = self.S_k_kr * (1 + ((nu_k_kr - 1) / (nu_k_kr + 1)) * self.d_korm / self.L_k_kr) * (
                1 + self.d_korm / self.L_k_kr)  # площадь крыльев
        L_kr = self.L_k_kr + self.d_korm  # размах крыльев
        l_kr = L_kr ** 2 / S_kr  # относительное удлинение
        nu_kr = nu_k_kr + self.d_korm / self.L_k_kr * (nu_k_kr - 1)
        z_a_kr = L_kr / 6 * ((nu_kr + 2) / (nu_kr + 1))  # расстояние от САХ до оси ЛА
        tan_05_kr = self.tan_1_kr + 2 / l_k_kr * (nu_k_kr - 1) / (nu_k_kr + 1)  # средняя стреловидность
        print(l_kr * tan_05_kr)
        b_a_kr = 4 / 3 * S_kr / L_kr * (1 - nu_kr / (nu_kr + 1) ** 2)  # САХ консоли
        z_a_k_kr = self.L_k_kr / 6 * (nu_k_kr + 2) / (nu_k_kr + 1)
        b__a_kr = b_a_kr / L_f  # относительное значение САХ консоли крыльев
        a_kr = 0.30  # ширина крыла у корпуса (меньшая грань трапеции)
        koef_kr = 1 / (1 - a_kr / self.b_kr)  # коэффициент перехода от ромбовидного к шестиугольному профилю крыла

        # вычисление формы рулей:

        l_k_op = self.L_k_op ** 2 / self.S_k_op
        nu_k_op = ((self.S_k_op / (self.L_k_op * self.b_op) - 0.5) ** (-1) / 2)  # относительное сужение крыльев
        b_1_op = self.b_op / nu_k_op  # концевая хорда крыльев
        b_a_k_op = 4 / 3 * self.S_k_op / self.L_k_op * (1 - (nu_k_op / (nu_k_op + 1) ** 2))  # САХ

        b_0_op = self.b_op * (1 + (nu_k_op - 1) / nu_k_op * self.d / self.L_k_op)  # бортовая хорда крыльев
        S_op = self.S_k_op * (1 + ((nu_k_op - 1) / (nu_k_op + 1)) * self.d / self.L_k_op) * (1 + self.d / self.L_k_op)  # площадь консолей крыльев
        L_op = self.L_k_op + self.d  # размах консолей крыльев
        l_op = L_op ** 2 / S_op  # относительное удлинение консолей
        nu_op = nu_k_op + self.d / L_op * (nu_k_op - 1)
        z_a_op = L_op / 6 * ((nu_op + 2) / (nu_op + 1))  # расстояние от САХ до оси ЛА
        tan_05_op = self.tan_0_op - 2 / l_k_op * (nu_k_op - 1) / (nu_k_op + 1)  # средняя стреловидность
        b_a_op = 4 / 3 * S_op / L_op * (1 - nu_op / (nu_op + 1) ** 2)  # САХ консоли
        z_a_k_op = self.L_k_op / 6 * (nu_k_op + 2) / (nu_k_op + 1)
        b__a_op = b_a_op / L_f  # относительное значение САХ консоли оперения
        c_op = 0.03  # относительная толщина профиля оперения
        a_op = 0.245  # ширина оперения у корпуса
        koef_op = 1 / (1 - a_op / self.b_op)  # коэффициент перехода от ромбовидного к шестиугольному профилю оперения

        # Вычисление координат крыльев и оперения

        L1_kr = L_f - self.L_hv_kr - self.b_kr / 2  # расстояние от носика корпуса до середины бортовой хорды крыла
        x_b_kr = L_f - self.L_hv_kr - self.b_kr  # координата начала бортовой хорды крыла
        # координата начала САХ крыла:
        if self.tan_1_kr == 0:
            x_b_a_kr = L_f - b_a_k_kr - self.L_hv_kr
        else:
            x_b_a_kr = L_f - b_a_k_kr - (z_a_k_kr - self.d / 2) / self.tan_1_kr - self.L_hv_kr

        x_b_op = self.L1_op - self.b_op / 2  # координата начала бортовой хорды оперения
        L_hv_op = L_f - self.L1_op + self.b_op / 2  # расстояние от конца бортовой хорды оперения до кормового среза

        # координата начала САХ оперения:

        if self.tan_0_op == 0:
            x_b_a_op = x_b_op
        else:
            x_b_a_op = x_b_op + (z_a_k_op - self.d / 2) / self.tan_0_op
        x_c_pl_ba = x_b_a_op + b_a_k_op / 2  # координата ЦТ площади передних консолей (середина САХ консолей)

        l_korp = L_f / self.d  # относительное удлинение корпуса

        nu_korm = self.d_korm / self.d  # относительное сужение кормовой части
        l_korm = self.L_korm / self.d  # относительное удлинение кормовой части

        W_nos = 0.074489
        # print("W_nos", W_nos)
        l_nos = self.L_nos / self.d  # относительное удлинение носовой части корпуса
        l_cil = l_korp - l_korm - l_nos  # относительное удлинение циллиндрической части корпуса
        x_otn_op_kr = 1.668 / b_a_k_kr  # относительное расстояние между оперением и средней хордой крыльев

        D_kr = self.d / L_kr  # относительный диаметр корпуса
        D_op = self.d / L_op  # относительный диаметр корпуса
        c_const_kr = (4 + 1 / nu_k_kr) * (1 + 8 * D_kr ** 2)  # коэффициент формы эпюры погонной нагрузки для крыльев
        c_const_op = (4 + 1 / nu_k_op) * (1 + 8 * D_op ** 2)  # коэффициент формы эпюры погонной нагрузки для оперения

        S_f = kk.pi * (self.d ** 2) / 4
        S__f = kk.pi * (self.d ** 2) / 4 / (kk.pi * (self.d ** 2) / 4)  # относительная площадь корпуса
        S__op = S_op / (kk.pi * (self.d ** 2) / 4)  # относительная площадь передних несущих поверхностей
        S__kr = S_kr / (kk.pi * (self.d ** 2) / 4)  # относительная площадь задних несущих поверхностей

        l_nos_ = (l_nos - r_ / 2) / kk.sqrt(1 - r_)  # относительное удлинение носовой части без затупления
        teta = kk.atan((1 - r_) / (l_nos - r_ / 2))  # угол наклона образующей носовой части (конуса)

        target = args[0]
        y_ti = target.y
        x_ti = target.x

        t = 0
        yt, xt, xx, yy, tt = [], [], [], [], []
        vv = []
        i = 0
        v_sr = 0

        cyy_nos = []
        cyy_kr = []
        cyy_op = []
        cyy1_alf = []
        cyy1_delt_op = []

        cxx_tr = []
        cxx_nos = []
        cxx_korm = []
        cxx_kr_pr = []
        cxx_op_pr = []
        cxx_kr_vol = []
        cxx_op_vol = []
        cxx_0f = []
        cxx_0_kr = []
        cxx_0_op = []
        cxx_0 = []
        f_xx = []
        f_yy = []
        cxx_zat = []
        cxx_con = []
        x_ffa_nos_cill = []
        x_ffa_f = []
        x_ffa_op = []
        x_ffa_kr = []
        da_ball = []
        m_zz_wz = []
        machh = []
        cyy_korm = []
        alff = []
        deltt = []
        delt_r = []
        # flag_v = []
        x_ffa = []
        m_zz_wz_f = []
        m_zz_wz_op = []
        m_zz_wz_kr = []

        cyy_sum = []
        cxx_sum = []

        f_yy_exp = []
        f_xx_exp = []

        fxx_t = []
        fyy_t = []
        n_max = 0

        omegg = []
        fifi = []

        # ((abs(target.y - self.y) > 5) or (abs(target.x - self.x) > 5)) and
        # while (t < 16):
        ind = 0
        jam = 0
        rast_min = kk.sqrt((target.y - self.y) ** 2 + (target.x - self.x) ** 2)
        rast_krit = kk.sqrt((target.y - self.y) ** 2 + (target.x - self.x) ** 2)
        while (rast_min >= 5) and (t <= 600) and (self.y >= 0):
            # flag_v.append(650)
            # self.delt = kk.asin()
            mach = self.v / Tabl.tab_atm(self.y, 2)
            machh.append(mach)

            cy_nos = Tabl.tab_3_3(mach, l_nos, l_cil)  # значение Су носовой части
            cyy_nos.append(cy_nos)

            cy_korm = -0.2 * 2 / 57.3 * (1 - nu_korm ** 2)
            cyy_korm.append(cy_korm)
            cy_kr = Tabl.tab_3_5(mach * kk.sqrt(Tabl.tab_3_22(mach, x_otn_op_kr)), l_kr, c_kr, tan_05_kr)
            cyy_kr.append(cy_kr)
            # if mach * kk.sqrt(Tabl.tab_3_21(mach, l_nos))<= 1.05:
            cy_op = Tabl.tab_3_5(mach * kk.sqrt(Tabl.tab_3_21(mach, l_nos)), l_op, c_op, tan_05_op)

            cyy_op.append(cy_op)
            cy1_alf_f = cy_nos + cy_korm

            K_aa_kr = 1 + 3 * D_kr - (D_kr * (1 - D_kr)) / nu_k_kr
            k_aa_kr = (1 + 0.41 * D_kr) ** 2 * ((1 + 3 * D_kr - 1 / nu_k_kr * D_kr * (1 - D_kr)) / (1 + D_kr) ** 2)
            eps_sr_alf = 0  # так как консоли передних и задних несущих поверхностей лежат на одной оси
            z_b = Tabl.tab_3_16(mach, l_op, nu_k_op, tan_05_op) # относительная координата вихря (по рис 3.16)
            i_ = 0  # Tabl.tab_3_17() # коэффициент интерференции вихрей
            if mach >= 1.3:
                fi_eps = 1  # отношение площади задней консоли, находящейся в конусе Маха, ко всей плозади консоли
            else:
                fi_eps = 0.3
            # eps_sr_alf = 57.3 / (2 * kk.pi) * i_ / z_b * L_k_op / L_k_kr * (cy1_alf_op / l_op) * k_aa_op / K_aa_kr * fi_eps
            cy1_alf_kr = (cy_kr * K_aa_kr) * (1 - eps_sr_alf)

            K_aa_op = 1 + 3 * D_op - (D_op * (1 - D_op)) / nu_k_op
            k_aa_op = (1 + 0.41 * D_op) ** 2 * ((1 + 3 * D_op - 1 / nu_k_op * D_op * (1 - D_op)) / (1 + D_op) ** 2)
            cy1_alf_op = cy_op * K_aa_op
            k_t_kr = Tabl.tab_3_21(mach, l_nos)
            k_t_op = Tabl.tab_3_22(mach, x_otn_op_kr)

            cy1_alf = cy1_alf_f * S__f + cy1_alf_kr * S__kr * k_t_kr + cy1_alf_op * S__op * k_t_op
            cyy1_alf.append(cy1_alf)

            K_delt_0_op = k_aa_op
            K_delt_0_kr = k_aa_kr
            k_delt_0_op = k_aa_op ** 2 / K_aa_op
            k_delt_0_kr = k_aa_kr ** 2 / K_aa_kr

            if mach <= 1:
                k_sh = 0.85
            elif (mach <= 1.4) and (mach > 1):
                k_sh = 0.85 + 0.14 * (mach - 1) / 0.4
            else:
                k_sh = 0.99
            n_ef = k_sh * kk.cos(0)
            cy1_delt_op = cy1_alf_op * K_delt_0_op * n_ef
            cyy1_delt_op.append(cy1_delt_op)

            ni_atm = Tabl.tab_atm(self.y, 5)
            re_f = self.v * L_f / ni_atm

            re_t = Tabl.tab_4_5(mach, re_f, 7, L_f)

            x_tt = re_t * ni_atm / self.v
            if x_tt >= (0.26 / L_f) or x_tt <= 0:
                x_tt = f_t / Ff
            # print(self.v, "V")
            # print(x_tt)
            cx_tr = Tabl.tab_4_2(re_f, x_tt) / 2 * Ff / S_f
            cxx_tr.append(cx_tr)

            cx_nos = Tabl.tab_4_12(mach, l_nos_)
            cxx_nos.append(cx_nos)

            cx_korm = Tabl.tab_4_24(mach, nu_korm, l_korm)
            cxx_korm.append(cx_korm)
            cx_0f = cx_tr + cx_nos + cx_korm
            cxx_0f.append(cx_0f)

            # профилное сопротивление несущих поверхностей
            re_k = self.v * self.b_kr / ni_atm
            re_k_t = Tabl.tab_4_5(mach, re_k, 5, self.b_kr)
            x_t_kr = re_k_t / re_k
            c_f_kr = Tabl.tab_4_2(re_k, x_t_kr)
            ni_c_kr = Tabl.tab_4_28(x_t_kr, c_kr)

            re_op = self.v * self.b_op / ni_atm
            re_op_t = Tabl.tab_4_5(mach, re_op, 5, self.b_op)
            x_t_op = re_op_t / re_op
            c_f_op = Tabl.tab_4_2(re_op, x_t_op)
            ni_c_op = Tabl.tab_4_28(x_t_op, c_op)

            cx_kr_pr = c_f_kr * ni_c_kr
            cxx_kr_pr.append(cx_kr_pr)
            cx_op_pr = c_f_op * ni_c_op
            cxx_op_pr.append(cx_op_pr)
            # волновое сопротивление несущих поверхностей
            if mach < 1.1:
                cx_op_vol = Tabl.tab_4_30(mach, nu_k_op, l_op, tan_05_op, c_op)
                cx_kr_vol = Tabl.tab_4_30(mach, nu_k_kr, l_kr, tan_05_kr, c_kr)
                if cx_kr_vol <= -0.5:
                    cx_kr_vol = 1
                if cx_op_vol <= -0.5:
                    cx_op_vol = 1
                cxx_kr_vol.append(cx_kr_vol)
                cxx_op_vol.append(cx_op_vol)
            else:

                cx_kr_vol = (Tabl.tab_4_30(mach, nu_k_kr, l_kr, tan_05_kr, c_kr)) * \
                            (1 + Tabl.tab_4_32(mach, tan_05_kr) * (koef_kr - 1))
                """if cx_kr_vol <= -2.5:
                    cx_kr_vol = 1"""

                cxx_kr_vol.append(cx_kr_vol)
                cx_op_vol = (Tabl.tab_4_30(mach, nu_k_op, l_op, tan_05_op, c_op)) * \
                            (1 + Tabl.tab_4_32(mach, tan_05_op) * (koef_op - 1))
                """if cx_op_vol <= -2.5:
                    cx_op_vol = 1"""
                cxx_op_vol.append(cx_op_vol)

            cx_0_op = cx_op_pr + cx_op_vol
            cx_0_kr = cx_kr_pr + cx_kr_vol
            cxx_0_kr.append(cx_0_kr)
            cxx_0_op.append(cx_0_op)

            cx_0 = 1.05 * (cx_0f * S__f + cx_0_op * k_t_op * S__op + cx_0_kr * k_t_kr * S__kr)
            cxx_0.append(cx_0)
            f_x = cx_0 * Tabl.tab_atm(self.y, 4) * self.v ** 2 * self.d ** 2 * kk.pi / 8
            f_xx.append(f_x)
            f_y = cy1_alf * Tabl.tab_atm(self.y, 4) * self.v ** 2 * self.d ** 2 * kk.pi / 8
            f_yy.append(f_y)
            # индуктивное сопротивление
            delt_cx1 = 2 * Tabl.tab_4_40(mach, l_nos, 1) * kk.sin(self.alf) ** 2
            cx_f_ind = cy1_alf_f * kk.sin(self.alf) + delt_cx1 * kk.cos(self.alf)
            D_0_op = K_aa_op - 57.3 * 0.2 * 0.3 * cy1_alf_op * k_aa_op ** 2
            D_1_op = k_aa_op * (1 - 57.3 * 0.2 * 0.3 * cy1_alf_op * k_delt_0_op * n_ef) + K_delt_0_op * n_ef
            D_2_op = k_delt_0_op * n_ef * (1 - 57.3 * 0.2 * 0.3 * cy1_alf_op * k_delt_0_op * n_ef)

            D_0_kr = K_aa_kr - 57.3 * 0.2 * 0.3 * cy1_alf_kr * k_aa_kr ** 2
            D_1_kr = -K_aa_kr + 2 * 57.3 * 0.2 * 0.3 * cy1_alf_kr * k_aa_kr
            D_2_kr = -57.3 * 0.2 * 0.3 * cy1_alf_kr * k_aa_kr

            if self.alf < 0.5:
                cx_op_ind = cy1_alf_op * (D_0_op + D_1_op * self.delt / 0.5 + D_2_op * (self.delt / 0.5) ** 2) * self.alf / 57.3
                cx_kr_ind = cy1_alf_kr * (D_0_kr + D_1_kr * self.delt / 0.5 + D_2_kr * (self.delt / 0.5) ** 2) * self.alf / 57.3
            else:
                cx_op_ind = cy1_alf_op * (D_0_op + D_1_op * self.delt / self.alf + D_2_op * (self.delt / self.alf) ** 2) * self.alf / 57.3
                cx_kr_ind = cy1_alf_kr * (D_0_kr + D_1_kr * self.delt / self.alf + D_2_kr * (self.delt / self.alf) ** 2) * self.alf / 57.3
            cx_ind = cx_f_ind * S__f + cx_op_ind * k_t_op * S__op + cx_kr_ind * k_t_kr * S__kr
            # cx_op_ind = cy

            # фокусы отдельных частей ЛА по углу атаки
            x_fa_nos_cill = self.L_nos - W_nos / S_f + Tabl.tab_5_7(mach, l_nos, l_cil, self.L_nos)
            x_ffa_nos_cill.append(x_fa_nos_cill)
            x_fa_korm = L_f - 0.5 * self.L_korm
            x_fa_f = 1 / cy1_alf_f * (cy_nos * x_fa_nos_cill + cy_korm * x_fa_korm)
            # print(cy_korm)
            x_ffa_f.append(x_fa_f)

            if mach < 1:
                x__iz_op = 0.21
                x__iz_kr = 0.15
            else:
                x__iz_op = 0.4
                x__iz_kr = 0.21

            x_f_iz_op = x_b_a_op + b_a_op * x__iz_op
            x_f_delt_op = x_f_iz_op - tan_05_op * Tabl.tab_5_11(D_op, self.L_k_op)
            x_f_iz_kr = x_b_a_kr + b_a_kr * x__iz_kr
            x_f_delt_kr = x_f_iz_kr - tan_05_kr * Tabl.tab_5_11(D_kr, self.L_k_kr)

            if mach > 1:
                b__b_op = self.b_op / (kk.pi / 2 * self.d * kk.sqrt(mach ** 2 - 1))
                b__b_kr = self.b_kr / (kk.pi / 2 * self.d * kk.sqrt(mach ** 2 - 1))
                L__hv_op = L_hv_op / (kk.pi * self.d * kk.sqrt(mach ** 2 - 1))
                L__hv_kr = self.L_hv_kr / (kk.pi * self.d * kk.sqrt(mach ** 2 - 1))
                F_1_op = 1 - 1 / (c_const_op * b__b_op ** 2) * (kk.e ** (-c_const_op * L__hv_op ** 2) - kk.e ** (-c_const_op * (b__b_op + L__hv_op) ** 2)) + kk.sqrt(kk.pi) / (b__b_op * kk.sqrt(c_const_op)) * Tabl.tab_int_ver(L__hv_op * kk.sqrt(2 * c_const_op))

                F_1_kr = 1 - 1 / (c_const_kr * b__b_kr ** 2) * (kk.e ** (-c_const_kr * L__hv_kr ** 2) - kk.e ** (-c_const_kr * (b__b_kr + L__hv_kr) ** 2)) + kk.sqrt(kk.pi) / (b__b_kr * kk.sqrt(c_const_kr)) * Tabl.tab_int_ver(L__hv_kr * kk.sqrt(2 * c_const_kr))

                F_op = 1 - kk.sqrt(kk.pi) / (2 * b__b_op * kk.sqrt(c_const_op)) * (Tabl.tab_int_ver((b__b_op + L__hv_op) * kk.sqrt(2 * c_const_op)) - Tabl.tab_int_ver(L__hv_op * kk.sqrt(2 * c_const_op)))
                F_kr = 1 - kk.sqrt(kk.pi) / (2 * b__b_kr * kk.sqrt(c_const_kr)) * (Tabl.tab_int_ver((b__b_kr + L__hv_kr) * kk.sqrt(2 * c_const_kr)) - Tabl.tab_int_ver(L__hv_kr * kk.sqrt(2 * c_const_kr)))

                x_f_b_op = x_f_iz_op + 0.02 * l_op * tan_05_op
                x_f_b_kr = x_f_iz_kr + 0.02 * l_kr * tan_05_kr

                x_f_ind_op = x_b_op + self.b_op * x_f_b_op * F_op * F_1_op
                x_f_ind_kr = x_b_kr + self.b_kr * x_f_b_kr * F_kr * F_1_kr
                x_fa_op = 1 / K_aa_op * (x_f_iz_op + (k_aa_op - 1) * x_f_delt_op + (K_aa_op - k_aa_op) * x_f_ind_op)
                x_fa_kr = 1 / K_aa_kr * (x_f_iz_kr + (k_aa_kr - 1) * x_f_delt_kr + (K_aa_kr - k_aa_kr) * x_f_ind_kr)
            else:
                x_f_b_op = x_f_iz_op + 0.02 * l_op * tan_05_op
                x_f_b_kr = x_f_iz_kr + 0.02 * l_kr * tan_05_kr
                x_f_ind_op = x_b_op + self.b_op * x_f_b_op
                x_f_ind_kr = x_b_kr + self.b_kr * x_f_b_kr
                x_fa_op = 1 / K_aa_op * (x_f_iz_op + (k_aa_op - 1) * x_f_delt_op + (K_aa_op - k_aa_op) * x_f_ind_op)
                x_fa_kr = 1 / K_aa_kr * (x_f_iz_kr + (k_aa_kr - 1) * x_f_delt_kr + (K_aa_kr - k_aa_kr) * x_f_ind_kr)

            x_ffa_op.append(x_fa_op)
            x_ffa_kr.append(x_fa_kr)

            x_fa = 1 / cy1_alf * ((cy1_alf_f * S__f * x_fa_f) + cy1_alf_op * S__op * x_fa_op * k_t_op + cy1_alf_kr * S__kr * x_fa_kr * k_t_kr)

            x_ffa.append(x_fa)

            # координаты фокуса рулей (передних консолей) по углам отклонения

            x_fd_op = 1 / K_delt_0_op * (k_delt_0_op * x_f_iz_op + (K_delt_0_op - k_delt_0_op) * x_f_ind_op)

            # Демпфирующие моменты АД поверхностей
            # x_c_ob - координата центра тяжести объема тела вращения
            x_c_ob = L_f * ((2 * (l_nos + l_cil)**2 - l_nos**2) / (4*(l_nos+l_cil) * (l_nos+l_cil - 2/3*l_nos)))
            m_z_wz_f = -2 * (1 - self.x_ct / L_f + (self.x_ct / L_f) ** 2 - x_c_ob / L_f)
            m_zz_wz_f.append(m_z_wz_f)

            x__ct_op = (self.x_ct - x_b_a_op) / b_a_op  # координата центра тяжести, измеренная от начала САХ рулей
            m_z_wz_op = -57.3 * (cy1_alf_op * (x__ct_op - 1 / 2) ** 2 * K_aa_op)
            m_zz_wz_op.append(m_z_wz_op)

            m_z_wz_delt_kr = -57.3 * (cy1_alf_kr * K_aa_kr) * eps_sr_alf * (self.x_ct - x_c_pl_ba) / b_a_kr * (self.x_ct - x_fa_kr) / b_a_kr
            x__ct_kr = (self.x_ct - x_b_a_kr) / b_a_kr
            m_z_wz_iz_kr = -57.3 * (cy1_alf_kr * (x__ct_kr - 1 / 2) ** 2 * K_aa_kr)
            m_z_wz_kr = m_z_wz_iz_kr * K_aa_kr + m_z_wz_delt_kr
            m_zz_wz_kr.append(m_z_wz_kr)

            m_z_wz = m_z_wz_f * S__f * L__f ** 2 + m_z_wz_op * S__op * b__a_op * kk.sqrt(k_t_op) + m_z_wz_kr * S__kr * b__a_kr * kk.sqrt(k_t_kr)
            m_zz_wz.append(m_z_wz)

            m_z_delt_op = cy1_delt_op * (self.x_ct - x_fd_op) / L_f

            m_z_alf = cy1_alf * (self.x_ct - x_fa) / L_f
            h_alf = x_fd_op - self.x_wr_op
            m_sharn_alf = -cy1_alf_op * h_alf / b_a_op
            h_delt = x_fd_op - self.x_wr_op
            m_sharn_delt = -cy1_delt_op * h_delt / b_a_op
            m_sharn = m_sharn_alf + m_sharn_delt
            da_bal = -m_z_alf / m_z_delt_op
            da_ball.append(da_bal)

            t += dt
            i += 1
            v_sr += self.v

            # функция наведения (Метод наведения и кинематические параметры перехвата)
            if kk.sqrt((y_ti - self.y) ** 2 + (x_ti - self.x) ** 2) >= 20000:
                par_1 = self.fi_viz
                self.fi_viz = kk.atan((y_ti - self.y) / (x_ti - self.x))
                self.d_fi_viz = (self.fi_viz - par_1) / dt

            else:
                par_1 = self.fi_viz
                self.fi_viz = kk.atan((target.y - self.y) / (target.x - self.x))
                self.d_fi_viz = (self.fi_viz - par_1) / dt
            fifi.append(self.fi_viz)
            omegg.append(self.omega)

            l_tra += self.v * dt
            # + self.alf - self.alf
            if (self.omega - self.fi_viz) >= - 0.1 and (self.omega - self.fi_viz) <= 0.1:
                self.delt_potr = self.fi_viz - self.omega
            elif (self.fi_viz - self.omega- self.alf) < 0:
                print(self.fi_viz, self.omega - self.alf)
                self.delt_potr = + delta_max
            else:
                self.delt_potr = - delta_max

            if (self.delt - self.delt_potr) <= 0:
                self.delt += d_delta_max * dt
            else:
                self.delt -= d_delta_max * dt

            deltt.append(self.delt_potr * 180 / kk.pi)
            delt_r.append(self.delt * 180 / kk.pi)

            n_y_a = kk.fabs(self.v * self.d_omega / g + kk.cos(self.omega))
            if kk.fabs(n_y_a) >= kk.fabs(n_max):
                n_max = n_y_a

            # print(self.omega * 180 / kk.pi,self.d_omega * 180 / kk.pi)
            # print(n_y_a)

            khi = cy1_delt_op * self.delt / cy1_alf
            # print(khi)
            q = Tabl.tab_atm(self.y, 4) * self.v ** 2 / 2
            M_z_wz = m_z_wz * q * S_f * L_f ** 2 / self.v
            delt_M_z = M_z_wz * self.wz
            # print(n_y_a * self.m * g)
            # print(cy1_alf * q * Sf * (1 + khi), self.p / 57.3)
            self.alf_potr = (n_y_a * self.m * g) / (cy1_alf * q * S_f * (1 + khi) + self.p / 57.3)
            # print(self.alf_potr)
            alff.append(self.alf_potr)
            # if self.alf < self.alf_potr:
            self.alf -= (1 / da_bal) * self.delt  # d_delta_max / 3 * dt
            # elif self.alf > self.alf_potr:
            #    self.alf += (1 / da_bal) * self.delt  # d_delta_max / 3 * dt
            """if self.alf <= 0:
                self.alf = 0"""
            cy_sum = cy1_alf * self.alf * kk.pi / 180 + cy1_alf + cy1_delt_op * self.delt * kk.pi / 180
            cyy_sum.append(cy_sum)
            cx_sum = cx_0  # + cx_ind
            cxx_sum.append(cx_sum)
            if jam <= cy1_alf_op * self.alf * kk.pi / 180 * self.v ** 2 \
                    * self.d ** 2 * kk.pi / 8 * Tabl.tab_atm(self.y, 4):
                jam = cy1_alf_op * self.alf * kk.pi / 180 * self.v ** 2 * self.d ** 2 \
                      * kk.pi / 8 * Tabl.tab_atm(self.y, 4)
            f_x_exp = cx_sum * Tabl.tab_atm(self.y, 4) * self.v ** 2 * self.d ** 2 * kk.pi / 8
            f_y_exp = cy_sum * Tabl.tab_atm(self.y, 4) * self.v ** 2 * self.d ** 2 * kk.pi / 8
            f_yy_exp.append(cy_sum * Tabl.tab_atm(self.y, 4) * self.v ** 2 * self.d ** 2 * kk.pi / 8)
            f_xx_exp.append(cx_sum * Tabl.tab_atm(self.y, 4) * self.v ** 2 * self.d ** 2 * kk.pi / 8)

            fxx_t.append(f_x_exp * dt)
            fyy_t.append(f_y_exp * dt)

            target.next_coord(t)
            xx.append(self.x)
            yy.append(self.y)
            xt.append(target.x)
            yt.append(target.y)
            vv.append(self.v)
            tt.append(t)

            self.p = self.p1
            # self.x_ct += dx_cm1 * dt
            # self.m -= dm1 * dt
            # self.v_ = 1 / self.m * (self.p - f_x_exp) - g * kk.sin(self.omega + self.alf)

            self = motion(self, f_x_exp, f_y_exp, target, a_m)
            """elif (t > t_st) and (t <= t_st + t_m):
                self.p = self.p2
                # self.x_ct += dx_cm2 * dt
                # self.m -= dm2 * dt
                self.v_ = 1 / self.m * (self.p * kk.cos(self.alf) - f_x_exp) - g * kk.sin(self.omega + self.alf)
            else:
                self.p = self.p3
                self.v_ = 1 / self.m * (self.p * kk.cos(self.alf) - f_x_exp) - g * kk.sin(self.omega + self.alf)"""

            """self.v1 = self.v
            self.v += self.v_ * dt
            self.x += (self.v + self.v1) / 2 * kk.cos(self.omega) * dt
            self.y += (self.v + self.v1) / 2 * kk.sin(self.omega) * dt"""
            # self.d_omega = 1 / (self.m * self.v) * ((self.p * kk.sin(self.alf) + f_y_exp) - self.m * g * kk.cos(self.omega))  # a_m * self.d_fi_viz
            # self.omega += self.d_omega * dt
            """if (kk.fabs(self.d_omega) <= d_omega_max):
                self.omega += self.d_omega * dt
            elif (self.d_omega >= 0):
                self.omega += d_omega_max * dt
            else:
                self.omega -= d_omega_max * dt"""

            rast_krit = kk.sqrt((target.y - self.y) ** 2 + (target.x - self.x) ** 2)
            # print(rast_krit)
            if rast_min >= rast_krit:
                rast_min = rast_krit
            # print(rast_krit)
            if ((ind * dt) >= 20) and ((ind * dt) <= 20.01):
                print(self.v)
            ind += 1
        print(dt * ind, self.v)

        if fin_ch <= 0:
            # print(sum(fxx_t), sum(fyy_t), sum(da_ball))
            krizzz = krit_maxi(self.m0, sum(fyy_t), sum(fxx_t), sum(da_ball))
            if krizzz >= 1000:
                krizzz = 0.1
            return krizzz
        else:

            """plt.plot(tt, f_yy_exp, 'r', label='Подъемная сила')
            plt.plot(tt, f_xx_exp, 'g', label='Сила лобового сопротивления')
            plt.grid(True)
            plt.legend()
            plt.xlabel('t, [c]')
            plt.ylabel('F, [H]')
            plt.show()

            plt.plot(tt, cyy_sum, 'r', label='Коэффициент подъемной силы')
            plt.plot(tt, cxx_sum, 'g', label='Коэффициент лобового сопротивления')
            plt.grid(True)
            plt.xlabel('t, [c]')
            plt.ylabel('Cy, Cx')
            plt.legend()
            plt.show()

            plt.plot(tt, da_ball)
            plt.ylabel('delta/alf')
            plt.xlabel('t, [с]')
            plt.grid(True)
            plt.show()

            plt.plot(x_ffa_op, tt, 'r', label='Координата фокуса оперения')
            plt.plot(x_ffa_kr, tt, 'g', label='Координата фокуса крыльев')
            plt.plot(x_ffa_f, tt, 'b', label='Координата фокуса корпуса')
            plt.plot(x_ffa, tt, 'y', label='Координата фокуса Ла')
            plt.grid(True)
            plt.axis([0, 4.2, 0, 250])
            plt.ylabel('t, [с]')
            plt.xlabel('x_fa, [м]')
            plt.legend()
            plt.show()"""
            print(l_tra)
            plt.plot(tt, omegg, 'r')
            plt.plot(tt, fifi, 'g')
            plt.show()

            plt.plot(xx, yy, 'r', label='Траектория ЛА')
            # plt.plot(xt, yt, 'g', label='Траектория цели')
            plt.plot(120000, 0,"x" 'g', label='Точка интереса')
            plt.grid(True)
            plt.xlabel('x, [м]')
            plt.ylabel('y, [м]')
            plt.legend()
            plt.show()

            plt.plot(xx, machh)
            plt.grid(True)
            plt.ylabel('M')
            plt.xlabel('t, [с]')
            plt.show()

            plt.plot(tt, machh)
            plt.grid(True)
            plt.ylabel('M')
            plt.xlabel('t, [с]')
            plt.show()

            plt.plot(tt, alff, 'r', label='Угол атаки')
            plt.plot(tt, deltt, 'g', label='Потребный угол отклоения рулей')
            plt.plot(tt, delt_r, 'b', label='Реальный угол отклонения рулей')
            plt.axis([0, 16, -15, 15])
            plt.xlabel('t, [c]')
            plt.ylabel('alf, delt, [град]')
            plt.legend()
            plt.grid(True)
            plt.show()

            plt.plot(tt, cyy_op, 'r', label='Cy_alf_op')
            plt.plot(tt, cyy_kr, 'g', label='Cy_alf_kr')
            plt.plot(tt, cyy1_alf, 'b', label='Cy1_alf')
            plt.plot(tt, cyy_korm, 'y', label='Cy_alf_korm')
            plt.plot(tt, cyy_nos, 'k', label='Cy_alf_nos')
            # plt.plot(tt, da_ball)
            # plt.plot(tt, m_zz_wz)
            # plt.axis([0, 16, -5, 5])
            # plt.plot(tt, cyy_op)
            # plt.plot(tt, cyy1_delt_op)
            plt.ylabel('Cy1_alf')
            plt.xlabel('t, [с]')
            plt.legend()
            plt.grid(True)
            plt.show()

            plt.plot(tt, cxx_nos, 'r', label='Сопротивление носовой части')
            plt.plot(tt, cxx_0f, 'g', label='Сумарное сопротивление корпуса')
            plt.plot(tt, cxx_korm, 'b', label='Сопротивление кормы')
            plt.plot(tt, cxx_tr, 'y', label='Сопротивление трению')
            plt.ylabel('Cx')
            plt.xlabel('t, [с]')
            plt.legend()
            plt.grid(True)
            plt.show()

            plt.plot(tt, cxx_kr_pr, 'r', label='Профильное сопротивление')
            plt.plot(tt, cxx_kr_vol, 'g', label='Волновое сопротивление')
            plt.plot(tt, cxx_0_kr, 'b', label='Суммарное сопротивление крыльев')
            plt.ylabel('Cx')
            plt.xlabel('t, [с]')
            plt.legend()
            plt.grid(True)
            plt.show()

            plt.plot(tt, cxx_op_pr, 'r', label='Профильное сопротивление')
            plt.plot(tt, cxx_op_vol, 'g', label='Волновое сопротивление')
            plt.plot(tt, cxx_0_op, 'b', label='Суммарное сопротивление оперения')
            plt.ylabel('Cx_0')
            plt.xlabel('t, [с]')
            plt.legend()
            plt.grid(True)
            plt.show()

            plt.plot(tt, cxx_0_op, 'r', label='Суммарное лобовое сопротивление')
            plt.plot(tt, cxx_0, 'g', label='Лобовое сопротивление оперения')
            plt.plot(tt, cxx_0_kr, 'b', label='Лобовое сопротивление крыльев')
            plt.plot(tt, cxx_0f, 'y', label='Лобовое сопротивление корпуса')
            plt.ylabel('Cx_0')
            plt.xlabel('t, [с]')
            plt.legend()
            plt.grid(True)
            plt.show()

            plt.plot(tt, m_zz_wz, 'r', label='Суммарный демпфирующий момент')
            plt.plot(tt, m_zz_wz_kr, 'g', label='Демпфирующий момент крыльев')
            plt.plot(tt, m_zz_wz_op, 'b', label='Демпфирующий момент оперения')
            plt.plot(tt, m_zz_wz_f, 'y', label='Демпфирующий момент корпуса')
            plt.ylabel('mz_wz')
            plt.xlabel('t, [с]')
            plt.legend()
            plt.grid(True)
            plt.show()

            plt.plot(tt, f_xx)
            plt.plot(tt, f_yy)
            plt.grid(True)
            plt.show()

            plt.plot(xx, yy)
            plt.plot(xt, yt)
            plt.grid(True)
            plt.show()
            plt.plot(tt, yy)
            plt.show()
            plt.plot(tt, xx)
            plt.show()
            plt.plot(tt, vv)
            # plt.plot(tt, flag_v, '--')
            plt.ylabel('V, [м/с]')
            plt.xlabel('t, [с]')
            plt.grid(True)
            plt.show()


def motion(*args):

    bomb = args[0]
    f_x_ex = args[1]
    f_y_ex = args[2]
    taret = args[3]
    a_m = args[4]
    # d_fi_viz = args[5]
    v_1 = bomb.v_
    bomb.v_ = 1 / bomb.m * (bomb.p - f_x_ex) - g * kk.sin(bomb.omega)
    v1 = bomb.v
    bomb.v += (bomb.v_ + v_1) / 2 * dt
    d_omega_1 = bomb.d_omega
    """bomb.d_omega = 1 / (bomb.m * bomb.v) * ((bomb.p * kk.sin(bomb.alf) + f_y_ex) - bomb.m * g * kk.cos(bomb.omega))
    bomb.omega += (bomb.d_omega + d_omega_1) / 2 * dt"""

    bomb.d_omega = a_m * bomb.d_fi_viz
    bomb.omega += bomb.d_omega * dt

    # bomb.w_z =

    bomb.y += (bomb.v + v1) / 2 * kk.sin(bomb.omega) * dt
    bomb.x += (bomb.v + v1) / 2 * kk.cos(bomb.omega) * dt

    return bomb


def krit_maxi(m_0_w, Sfy, Sfx, dealf, parame):
    m_0_w_0 = 7.3
    """pp1 = [0.5289038465393716]
    pp2 = [0.4127488674733952]
    pp3 = [5.547746942079459]  # среднее значение отношения delta/alf"""
    pipi = [[5776, 4505, 6056], [4581, 3706, 4899], [3783, 3169, 4130], [2776, 2132, 2855],
            [3066, 2332, 3159], [8559, 6684, 9607]]
    k_paramz = 0
    return (m_0_w / m_0_w_0) ** 0 * (Sfy / pipi[parame][1]) ** 2 * \
           (Sfx / pipi[parame][0]) ** (-4) * (dealf / pipi[parame][2]) ** (-1)


targ = Target(120000, 0, 0)

dt = 10 ** -3
g = 9.80665

igla = Rocket(targ.y, targ.x)
print(igla.navigation(targ))



