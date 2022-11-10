import math as kk
import Tabl as tb
import matplotlib.pyplot as plt


class Munition(object):

    def __init__(self, *args):

        self.x = args[0][0]
        self.y = args[0][1]

        self.v = args[1]
        self.mah = self.v / (tb.tab_atm(self.y, 2))

        self.tetta = args[2]
        self.w_z = 0
        self.d_w_z_dt = 0

        self.nu = 0 * kk.pi / 180

        self.alf = self.nu - self.tetta

        # массовые параметры

        self.m = 495  # масса
        self.I_z = 413.15  # момент инерции
        self.x_cm = 1.925  # координата центра тяжести
        self.x_c_ob = 1.92  # координата центра тяжести объема тела вращения

        # геометрические параметры

        self.d = 0.49  # диаметр миделя
        self.S_f = self.d ** 2 # kk.pi * self.d ** 2 / 4  # площадь миделева сечения
        self.L_F = 4.06  # длина корпуса
        self.L_F_ = self.L_F / self.L_F
        self.L_cill = 1.799  # длина цилиндрической части
        self.F_f = 6.4  # площадь обтекаемой потоком поверхности м^2

        #  передняя консоль
        self.krit_r_1 = 0  # маркер управляющей поверхности 0 - статика, 1 - руль
        self.delt_1 = 0

        self.d_1 = self.d  # диаметр корпуса в по координате серидины бортовой хорды
        self.L_1 = 2.7  # размах передней консоли
        self.L_k_1 = self.L_1 - self.d_1  # размах изолированной консоли
        self.S_k_1 = 0.74 # 0.739776  # площадь изолированных консолей
        self.L_hv_1 = 2.222  # расстояние от конца бортовой хорды до кормового среза

        self.sum_S_1_ = self.S_k_1 * 2 / self.S_f  # суммарная относительная площадь
        self.b_b_1 = 0.310  # бортовая хорда
        self.b_1_1 = 0.300  # концевая хорда
        self.tan_0_1 = 0.466
        self.tan_1_1 = 0.456  # тангенс угла задней стреловидности
        # self.tan_05_1 = 0  # тангенс угла средней стреловидности

        self.l_k_kr_1 = kk.sqrt(self.L_k_1 ** 2 / self.S_k_1)  # удлинение консоли передней консоли

        self.eta_1_k = self.b_b_1 / self.b_1_1  # сужение консоли
        self.b_a_k_1 = 4 / 3 * self.S_k_1 / self.L_k_1 * (1 - self.eta_1_k / (self.eta_1_k + 1) ** 2)
        self.b_a_k_1_ = self.b_a_k_1 / self.L_F
        self.tan_05_1 = self.tan_0_1 - 2 / self.l_k_kr_1 * (self.eta_1_k - 1) / (self.eta_1_k + 1)

        self.L_1_1 = self.L_F - self.L_hv_1 - self.b_b_1 / 2  # расстояние от носика корпуса до середины бортовой хорды
        self.x_b_1 = self.L_1_1 - self.b_b_1 / 2  # координата начала бортовой хорды
        z_a_k_kr = self.L_k_1 / 6 * ((self.eta_1_k - 1) / (self.eta_1_k + 1))
        if self.tan_1_1 == 0:
            self.x_ak_1 = self.L_F - self.b_a_k_1 - self.L_hv_1
        else:
            self.x_ak_1 = self.L_F - self.b_a_k_1 - (z_a_k_kr - self.d_1 / 2) / self.tan_1_1 - self.L_hv_1
        self.L_1_1_ = self.L_1_1 / self.d_1
        self.d_1_ = self.d_1 / self.L_1
        self.c_1 = 0.02  # относительная толщина передней консоли

        #  задняя консоль
        self.krit_r_2 = args[3]  # 1  # маркер управляющей поверхности 0 - статика, 1 - руль
        self.delt_2 = args[4]   # -2

        self.d_2 = 0.216  # диаметр корпуса в по координате серидины бортовой хорды
        self.L_2 = 0.719
        self.L_k_2 = self.L_2 - self.d_2
        self.S_k_2 = 0.1317
        self.sum_S_2_ = self.S_k_2 * 2 / self.S_f  # суммарная относительная площадь
        self.b_b_2 = 0.267
        self.b_1_2 = 0.256
        self.L_hv_2 = 0.148
        self.tan_0_2 = 0.5095
        self.tan_1_2 = 0.466

        self.c_2 = 0.02
        self.d_2_ = self.d_2 / self.L_2

        self.l_k_kr_2 = kk.sqrt(self.L_k_2 ** 2 / self.S_k_2)  # удлинение консоли
        self.eta_2_k = self.b_b_2 / self.b_1_2  # сужение консоли
        self.b_a_k_2 = 4 / 3 * self.S_k_2 / self.L_k_2 * (1 - self.eta_2_k / (self.eta_2_k + 1) ** 2)
        self.b_a_k_2_ = self.b_a_k_2 / self.L_F
        self.tan_05_2 = self.tan_0_2 - 2 / self.l_k_kr_2 * (self.eta_2_k - 1) / (self.eta_2_k + 1)

        self.x_1_2 = 1.527  # расстояние от конца САХ передней консоли до середины бортовой задней
        self.x_1_2_ = self.x_1_2 / self.b_a_k_1
        self.L_1_2 = self.L_F - self.L_hv_2 - self.b_b_2  # расстояние от носика корпуса до середины бортовой хорды
        self.x_b_2 = self.L_1_2 - self.b_b_2 / 2
        z_a_k_kr = self.L_k_2 / 6 * ((self.eta_2_k - 1) / (self.eta_2_k + 1))
        if self.tan_1_2 == 0:
            self.x_ak_2 = self.L_F - self.b_a_k_2 - self.L_hv_2
        else:
            self.x_ak_2 = self.L_F - self.b_a_k_2 - (z_a_k_kr - self.d_2 / 2) / self.tan_1_2 - self.L_hv_2  # координата начала САХ
        self.L_1_2_ = self.L_1_2 / self.d_1

        self.x_t_1_ = (self.x_cm - self.x_ak_1) / self.b_a_k_1  # плечо от цм до САХ 1 консоли, измеренная от начала САХ
        self.x_t_2_ = (self.x_cm - self.x_ak_2) / self.b_a_k_2
        # в долях САХ стр. 277

        self.L_nos = 0.677

        self.d_korm = 0.1675  # диаметр кормового среза
        self.S_dn = self.d_korm ** 2 / 4 * kk.pi
        self.eta_korm = self.d_korm / self.d
        self.L_korm = self.L_F - self.L_nos - self.L_cill  # длина кормовой части
        print(self.L_korm)

        # удлинения корпуса
        self.l_nos = self.L_nos / self.d
        self.l_cill = self.L_cill / self.d  # L_F
        self.l_korm = self.L_korm / self.d  # L_F
        self.l_f = self.L_F / self.d  # L_F

        # объем носовй части

        self.W_nos = 0.533 * self.L_nos * self.S_f
        self.W_korm = self.L_korm * (self.S_f + self.S_dn) / 2

        # относительные площади
        self.S_f_ = self.S_f / self.S_k_1
        self.S_1 = self.S_k_1 * (1 + (self.eta_1_k - 1) / (self.eta_1_k + 1) * self.d_1 / self.L_k_1) * (1 + self.d_1 / self.L_k_1)
        self.S_2 = self.S_k_2 * (1 + (self.eta_2_k - 1) / (self.eta_2_k + 1) * self.d_2 / self.L_k_2) * (1 + self.d_2 / self.L_k_2)
        print(self.S_1, self.S_k_1)
        self.S_1_ = self.S_1 / self.S_k_1
        self.S_2_ = self.S_2 / self.S_k_1

    def dynamic(self, x_a, y_a, M_z):

        d_v_dt = (-x_a - self.m * g * kk.sin(self.tetta)) / self.m
        d_tetta_dt = (y_a - self.m * g * kk.cos(self.tetta)) / (self.m * self.v)
        self.d_w_z_dt = M_z / self.I_z
        d_nu_dt = self.w_z
        d_m_dt = 0
        d_y_dt = self.v * kk.sin(self.tetta)
        d_x_dt = self.v * kk.cos(self.tetta)

        self.v += d_v_dt * dt
        self.mah = self.v / (tb.tab_atm(self.y, 2))

        self.tetta += d_tetta_dt * dt
        self.w_z += self.d_w_z_dt * dt
        self.nu += d_nu_dt * dt
        self.alf = (self.nu - self.tetta) * 180 / kk.pi

        self.y += d_y_dt * dt
        self.x += d_x_dt * dt
        self.m += d_m_dt * dt

    def aero(self, *args):

        q = tb.tab_atm(self.y, 4) * self.v ** 2 / 2
        """
        ______________________________________________________________________
        Коэффициент подъемной силы
        ______________________________________________________________________
        """
        # подъемная сила
        # корпус Су
        c_y_1_a_korm = -0.2 * 2 / 57.3 * (1 - self.eta_korm ** 2)
        c_y_1_a_nos = tb.tab_3_3(self.mah, self.l_nos, self.l_cill)
        #
        # print(self.mah)
        # cyy.append(c_y_1_a_nos)
        c_y_1_a_f = c_y_1_a_nos + c_y_1_a_korm

        # передняя консоль
        c_y_1_kr_a_1 = self.l_k_kr_1 * tb.tab_3_5(self.mah, self.l_k_kr_1, self.c_1, self.tan_05_1)
        K_aa_x_1 = 1 + 3 * self.d_1_ - (self.d_1_ * (1 - self.d_1_) / self.eta_1_k)
        k_aa_x_1 = (1 + 0.41 * self.d_1_) ** 2 * (K_aa_x_1 / (1 + self.d_1_) ** 2)
        delta_x_1 = (0.093 / (self.v * self.L_1_1 / tb.tab_atm(self.y, 5)) ** (1 / 5) *
                     self.L_1_1 / self.d_1 * (1 + 0.4 * self.mah + 0.147 * self.mah ** 2 - 0.006 * self.mah ** 3))
        khi_ps_1 = ((1 - (2 * self.d_1_ ** 2) / (1 - self.d_1_ ** 2) * delta_x_1) *
                    (1 - (self.d_1_ * (self.eta_1_k - 1)) / ((1 - self.d_1_) * (self.eta_1_k + 1)) * delta_x_1))
        # изменения нормальной силы вызванные пограничным слоем
        khi_m_1 = 1  # при дозвуке, иначе по таблице 3.13
        khi_nos_1 = 0.6 + 0.4 * (1 - kk.e ** (-0.5 * self.L_1_1_))
        K_aa_1 = K_aa_x_1 * khi_ps_1 * khi_m_1 * khi_nos_1
        k_aa_1 = k_aa_x_1 * khi_ps_1 * khi_m_1 * khi_nos_1
        c_y_1_a_1 = c_y_1_kr_a_1 * K_aa_1

        k_t_1 = tb.tab_3_21(self.mah, self.l_nos)

        if self.krit_r_1 == 1:
            c_y_delt_1_1 = c_y_1_kr_a_1
            c_y_delt_1_2 = c_y_1_kr_a_1
            c_y_delt_1 = c_y_1_kr_a_1
        else:
            c_y_delt_1 = 0

            # задняя консоль

        c_y_1_kr_a_2 = self.l_k_kr_2 * tb.tab_3_5(self.mah, self.l_k_kr_2, self.c_2, self.tan_05_2)
        K_aa_x_2 = 1 + 3 * self.d_2_ - (self.d_2_ * (1 - self.d_2_) / self.eta_2_k)
        k_aa_x_2 = (1 + 0.41 * self.d_2_) ** 2 * (K_aa_x_2 / (1 + self.d_2_) ** 2)
        delta_x_2 = (0.093 / (self.v * self.L_1_2 / tb.tab_atm(self.y, 5)) ** (1 / 5) *
                     self.L_1_2 / self.d_2 * (1 + 0.4 * self.mah + 0.147 * self.mah ** 2 - 0.006 * self.mah ** 3))

        khi_ps_2 = ((1 - (2 * self.d_2_ ** 2) / (1 - self.d_2_ ** 2) * delta_x_2) *
                    (1 - (self.d_2_ * (self.eta_2_k - 1)) / ((1 - self.d_2_) * (self.eta_2_k + 1)) * delta_x_2))
        # изменения нормальной силы вызванные пограничным слоем
        khi_m_2 = 1  # при дозвуке, иначе по таблице 3.13
        khi_nos_2 = 0.6 + 0.4 * (1 - kk.e ** (-0.5 * self.L_1_2_))
        K_aa_2 = K_aa_x_2 * khi_ps_2 * khi_m_2 * khi_nos_2
        k_aa_2 = k_aa_x_2 * khi_ps_2 * khi_m_2 * khi_nos_2
        c_y_1_a_2 = c_y_1_kr_a_2 * K_aa_2
        k_t_2 = tb.tab_3_21(self.mah, self.l_nos) * tb.tab_3_22(self.mah, self.x_1_2_)

        if self.krit_r_2 == 1:
            K_delt_0_x_2 = k_aa_x_2
            k_delt_0_x_2 = k_aa_x_2 ** 2 / K_aa_x_2
            khi_ps_2_ = (1 - self.d_2_ * (1 + delta_x_2)) * (
                        1 - (self.eta_2_k - 1) / (self.eta_2_k + 1 - 2 * self.d_2_) * self.d_2_ * (1 + delta_x_2)) / (
                                    (1 - self.d_2_) * (
                                        1 - (self.eta_2_k - 1) / (self.eta_2_k + 1 - 2 * self.d_2_) * self.d_2_))
            k_delt_0_2 = k_delt_0_x_2 * khi_ps_2_ * khi_m_2
            K_delt_0_2 = (k_delt_0_x_2 + (K_delt_0_x_2 - k_delt_0_x_2) * 1) * khi_ps_2_ * khi_m_2
            k_sh = 0.82
            cos_hi_p = 1
            n_2 = k_sh * cos_hi_p
            c_y_delt_2_2 = c_y_1_kr_a_2 * K_delt_0_2 * n_2 * self.S_2_ * k_t_2
            c_y_delt_2 = c_y_delt_2_2
        else:
            c_y_delt_2 = 0
            k_delt_0_2 = 0
            K_delt_0_2 = 0
            n_2 = 0

        c_y_delt_2 = c_y_delt_2  # * kk.sqrt(2)   # крестокрыльный аппарат

        c_y_1_a = c_y_1_a_f * self.S_f_ + c_y_1_a_2 * self.S_2_ * k_t_2  + c_y_1_a_1 * self.S_1_ * k_t_1
        c_y_1 = c_y_1_a * self.alf  + c_y_delt_2 * self.delt_2

        """
        ______________________________________________________________________
        Коэффициент силы сопротивления
        ______________________________________________________________________
        """
        # сила сопротивления
        # сила сопротивления трению
        rei_f = self.v * self.L_F / tb.tab_atm(self.y, 5)
        x_t_ = 0
        x2cf = tb.tab_4_3(self.mah, x_t_) * tb.tab_4_2(rei_f, x_t_)

        cx_tr = (x2cf / 2) * (self.F_f / self.S_f)

        cx_nos = tb.tab_4_12(self.mah, self.l_nos)

        cx_korm = tb.tab_4_24(self.mah, self.eta_korm, self.l_korm)
        # print(x2cf, self.mah, rei_f, self.v, tb.tab_atm(self.y, 5), 'x2cf', tb.tab_4_2(rei_f, x_t_))
        p_dn = 0.0155 / kk.sqrt(self.l_f * x2cf / 2)
        k_eta = self.eta_korm
        cx_dn = (-p_dn) * k_eta * self.S_dn / self.S_f

        cx0_f = cx_tr + cx_nos + cx_korm #+ cx_dn

        # профильное сопротивление передней несущей поверхности
        mah_1 = self.mah * kk.sqrt(k_t_1)
        rei_1 = (mah_1 * tb.tab_atm(self.y, 2)) * self.b_a_k_1  # САХ консоли
        x2cf_1 = tb.tab_4_3(mah_1, x_t_) * tb.tab_4_2(rei_1, x_t_)
        eta_c = tb.tab_4_28(x_t_, self.c_1)
        cx_p_1 = x2cf_1 * eta_c

        # волновое сопротивление передней несущей поверхности

        if mah_1 <= 0.65:  # критическое число Маха определить по рис. 4.34
            cx_v_1 = 0
        else:
            cx_v_1 = tb.tab_4_30(mah_1, self.eta_1_k, self.l_k_kr_1, self.tan_05_1,
                                 self.c_1) * self.l_k_kr_1 * self.c_1 ** 2

        cx01 = cx_p_1 + cx_v_1

        # профильное сопротивление задней несущей поверхности
        mah_2 = self.mah * kk.sqrt(k_t_2)
        rei_2 = (mah_2 * tb.tab_atm(self.y, 2)) * self.b_a_k_2  # САХ консоли

        x2cf_2 = tb.tab_4_3(mah_2, x_t_) * tb.tab_4_2(rei_2, x_t_)
        eta_c = tb.tab_4_28(x_t_, self.c_2)
        cx_p_2 = x2cf_2 * eta_c

        # волновое сопротивление задней несущей поверхности

        if mah_2 <= 0.65:  # критическое число Маха определить по рис. 4.34
            cx_v_2 = 0
        else:
            cx_v_2 = tb.tab_4_30(mah_2, self.eta_2_k, self.l_k_kr_2, self.tan_05_2, self.c_2) * self.l_k_kr_2 * self.c_2 ** 2

        cx02 = cx_p_2 + cx_v_2

        cx0 = 1.05 * (cx0_f * self.S_f_ + cx02 * k_t_2 * self.sum_S_2_+  cx01 * k_t_1 * self.sum_S_1_)

        # индукционная сила сопротивления

        cx_ind_f = (57.3 * c_y_1_a_f + 2 * tb.tab_4_40(self.mah, self.l_nos, 0)) * (self.alf / 57.3) ** 2

        # 1 kons
        if self.alf == 0:
            da_1 = 0
        else:
            da_1 = self.delt_1 / self.alf
        shi_1 = 0.8  # tb.tab_4_43()
        c_f_1 = 1 / (57.3 * c_y_1_kr_a_1) - 1 / (kk.pi * self.l_k_kr_1)

        D_0_1 = K_aa_1 - 57.3 * shi_1 * c_f_1 * c_y_1_kr_a_1 * k_aa_1 ** 2

        cx_ind_1 = c_y_1_kr_a_1 * D_0_1 * (self.alf ** 2 / 57.3)

        # 2 kons
        if self.alf == 0:
            da_2 = 0
        else:
            da_2 = self.delt_2 / self.alf
        shi_2 = 0.8  # tb.tab_4_43()
        c_f_2 = 1 / (57.3 * c_y_1_kr_a_2) - 1 / (kk.pi * self.l_k_kr_2)
        # z_v_ po tabl_3_16
        y_v = 0
        ii = 1
        z_v_ = tb.tab_3_16(mah_2, self.l_k_kr_1, self.eta_1_k, self.tan_05_1)
        psi_eps = 1
        eps_sr_alf = 0  # 57.3 / 2 * kk.pi * ii / z_v_ * self.l_k_kr_1 / self.l_k_kr_2 * (c_y_1_kr_a_1 /
        # self.l_k_kr_2) * k_aa_1 / K_aa_2 * psi_eps

        D_0_2 = (K_aa_2 - 57.3 * shi_2 * c_f_2 * c_y_1_kr_a_2 * k_aa_2 ** 2 * (1 - eps_sr_alf))
        D_1_2 = k_aa_2 * (1 - eps_sr_alf) * (1 - 2 * 57.3 * shi_2 * c_f_2 * c_y_1_kr_a_2 * k_delt_0_2 * n_2) + K_delt_0_2 * n_2
        D_2_2 = k_delt_0_2 * n_2 * (1 - 57.3 * shi_2 * c_f_2 * c_y_1_kr_a_2 * k_delt_0_2 * n_2)
        cx_ind_2 = c_y_1_kr_a_2 * (D_0_2 + D_1_2 * da_2 + D_2_2 * da_2 ** 2) * (self.alf ** 2 / 57.3)

        cx_ind = cx_ind_f * self.S_f_ + cx_ind_2 * self.S_2_ + cx_ind_1 * self.S_1_ * k_t_1

        c_x_1_a = cx0 + cx_ind

        """
        ______________________________________________________________________
        Коэффициент аэродинамического момента тангажа
        ______________________________________________________________________
        """

        """
        ______________________________________________________________________
        коэффициент аэродинамического момента при углах и угловых скоростях, 
        равных нулю
        ______________________________________________________________________
        """

        m_z_1_0 = 0

        x_f_a_nos = self.L_nos - self.W_nos / self.S_f
        x_f_a_korm = self.L_F - (self.S_f * self.L_korm - self.W_korm) / (self.S_f - self.S_dn)
        x_f_a_f = 1 / c_y_1_a_f * (c_y_1_a_nos * x_f_a_nos + c_y_1_a_korm * x_f_a_korm)

        x_f_kr_1_ = tb.tab_5_8(mah_1, self.l_k_kr_1, self.tan_05_1, self.eta_1_k)
        x_f_b_1_ = x_f_kr_1_ + 0.02 * self.l_k_kr_1 * self.tan_05_1
        x_f_i_a_1 = self.x_b_1 + self.b_b_1 * x_f_b_1_

        x_f_kr_1 = self.x_ak_1 + self.b_a_k_1 * x_f_kr_1_

        f_1_1 = tb.tab_5_11(self.d_1_, self.l_k_kr_1)

        delt_x_f_1 = x_f_kr_1 - f_1_1 * self.tan_05_1

        x_f_a_1 = 1 / K_aa_1 * (x_f_kr_1 + (k_aa_1 - 1) * delt_x_f_1 + (K_aa_1 - k_aa_1) * x_f_i_a_1)

        x_f_kr_2_ = tb.tab_5_8(mah_2, self.l_k_kr_2, self.tan_05_2, self.eta_2_k)
        x_f_b_2_ = x_f_kr_2_ + 0.02 * self.l_k_kr_2 * self.tan_05_2
        x_f_i_a_2 = self.x_b_2 + self.b_b_2 * x_f_b_2_

        x_f_kr_2 = self.x_ak_2 + self.b_a_k_2 * x_f_kr_2_

        f_1_2 = tb.tab_5_11(self.d_2_, self.l_k_kr_2)

        delt_x_f_2 = x_f_kr_1 - f_1_2 * self.tan_05_2

        x_f_a_2 = 1 / K_aa_2 * (x_f_kr_2 + (k_aa_2 - 1) * delt_x_f_2 + (K_aa_2 - k_aa_2) * x_f_i_a_2)

        x_f_a = 1 / c_y_1_a * (c_y_1_a_f * x_f_a_f * self.S_f_ + c_y_1_a_1 * x_f_a_1 * self.S_1_ + c_y_1_a_2 * x_f_a_2 *
                               self.S_2_)
        m_z_1_alf = c_y_1_a * (self.x_cm - x_f_a) / self.L_F

        x_f_delt_1 = 0
        m_z_1_delt_1 = 0  # c_y_delt_1 * (self.x_cm - x_f_delt_1) / self.L_F
        if self.delt_2 == 0:
            m_z_1_delt_2 = 0
        else:
            # print(K_delt_0_2)
            x_f_delt_2 = 1 / K_delt_0_2 * (k_delt_0_2 * x_f_a_2 + (K_delt_0_2 - k_delt_0_2) * x_f_i_a_2)
            m_z_1_delt_2 = c_y_delt_2 * (self.x_cm - x_f_delt_2) / self.L_F
            # print(x_f_delt_2, m_z_1_delt_2)


        """
        Коэффициент демпфирующего момента
        """

        m_z_wz_1_f = -2 * (1 - self.x_cm / self.L_F + (self.x_cm / self.L_F) ** 2 - self.x_c_ob / self.L_F)

        m_z_wz_c_y_a_1 = tb.tab_5_15(self.l_k_kr_1, self.tan_05_1) - tb.tab_5_16(self.l_k_kr_1, self.tan_05_1) * (0.5 - self.x_t_1_) - 57.3 * (0.5 - self.x_t_1_) ** 2
        m_z_wz_1_1 = m_z_wz_c_y_a_1 * c_y_1_kr_a_1 * K_aa_1
        # m_z_wz_1_1 = -57.3 * (c_y_1_kr_a_1 * (self.x_t_1_ - 0.5) ** 2 * K_aa_1)
        m_z_wz_c_y_a_2 = tb.tab_5_15(self.l_k_kr_2, self.tan_05_2) - tb.tab_5_16(self.l_k_kr_2, self.tan_05_2) * (0.5 - self.x_t_2_) - 57.3 * (0.5 - self.x_t_2_) ** 2
        m_z_wz_1_2 = m_z_wz_c_y_a_2 * c_y_1_kr_a_2 * K_aa_2
        # m_z_wz_1_2 = -57.3 * ()

        m_z_wz_1 = m_z_wz_1_f * self.L_F_ ** 2 * self.S_f_ + m_z_wz_1_1 * self.S_1_ * self.b_a_k_1_ ** 2 * kk.sqrt(k_t_1) + m_z_wz_1_2 * self.S_2_ * self.b_a_k_2_ ** 2 * kk.sqrt(k_t_2)
        w_z_ = self.w_z * self.L_F / self.v

        m_z_1 = m_z_1_0 + m_z_1_alf * self.alf + m_z_1_delt_1 * self.delt_1 + m_z_1_delt_2 * self.delt_2 + m_z_wz_1 * w_z_  # + m_z_alf__1 * alf_ + m_z_delt__1 * delt_

        # print(c_x_1_a, c_y_1_a)

        c_x = c_x_1_a * kk.cos(self.alf/ 57.3) # * kk.cos(self.alf) + c_y_1 * kk.sin(self.alf)
        c_y = c_y_1 * kk.cos(self.alf / 57.3)  + c_x_1_a * kk.sin(self.alf / 57.3)  # c_y_1_a * kk.cos(self.alf) + c_x_1_a * kk.sin(self.alf)
        x_aer = c_x * q * self.S_k_1  # * self.alf
        y_aer = c_y * q * self.S_k_1  # * self.alf
        m_aer = m_z_1 * q * self.S_k_1 * self.L_F
        mzz.append(m_z_1)
        cxx.append(x_aer)
        cyy.append(y_aer/x_aer)
        # print(x_aer, y_aer, g * self.m * kk.cos(self.tetta))

        return x_aer, y_aer, m_aer

    def navi(self, *args):

        gg = 10 * args[0]
        return gg


dt = 4 * 10 ** -2
g = 9.80665

start = [0, 12 * 10 ** 3]
mah_0 = 0.8
velocity = tb.tab_atm(start[1],2) * mah_0

bomb = Munition(start, velocity, 0, 0,0)
jss = 0
xx_ = [[], [], [], [], [], [], [], [], []]
yy_ =[[], [], [], [], [], [], [], [], []]
aa_ = [[], [], [], [], [], [], [], [], []]
mzz_ = [[], [], [], [], [], [], [], [], []]
cyy_ = [[], [], [], [], [], [], [], [], []]
cxx_ = [[], [], [], [], [], [], [], [], []]
vv_ = [[], [], [], [], [], [], [], [], []]
ti_ = [[], [], [], [], [], [], [], [], []]

# p_d_2 = [1, 1, 1, 1, 1, 1, 0, 1, 1]
# g_d_2 = [-15, -13, -10, -7, -4, -2, 0, 2, 4]
p_d_2 = [1, 1, 1, 1, 1, 1, 0, 1, 1]
g_d_2 = [-10, -9, -8, -7, -4, -2, 0, 2, 4]
color = ['g', 'b', 'r', 'y', 'k', 'pink', 'm', 'lightpink', 'honeydew']

dm = 4

for dv in range(0, dm):
    bomb = Munition(start, velocity, 0, p_d_2[dv], g_d_2[dv])

    xx = []
    yy = []
    aa = []
    mzz = []
    cyy = []
    cxx = []
    vv = []
    ti = []
    jss = 0

    while (jss<=20000) and bomb.y >=0 and bomb.mah <= 6 and bomb.x >=0:
        X_a, Y_a, m_a = bomb.aero()
        bomb.dynamic(X_a, Y_a, m_a)
        # print(bomb.v, bomb.y, bomb.x, X_a, Y_a, bomb.m * g)
        xx.append(bomb.x)
        yy.append(bomb.y)
        vv.append(bomb.mah)
        aa.append(bomb.alf)
        ti.append(dt * jss)
        jss+=1
    print(bomb.x)
    xx_[dv] = xx
    yy_[dv] = yy
    aa_[dv] = aa
    mzz_[dv] = mzz
    cyy_[dv] = cyy
    cxx_[dv] = cxx
    vv_[dv] = vv
    ti_[dv] = ti


'''print(ti, 'time')
print(dt * jss)
print(bomb.y, bomb.x, 'point')'''
for i in range(0, dm):
    plt.plot(ti_[i], vv_[i], color[i], label='dv = ' + str(g_d_2[i]))

plt.xlabel('t, c')
plt.ylabel('М')
plt.legend()
plt.grid(True)
plt.axis([0, 70, 0, 1.1])
plt.show()
for i in range(0, dm):
    plt.plot(xx_[i], yy_[i], color[i], label='dv = ' + str(g_d_2[i]))

plt.xlabel('X, м')
plt.ylabel('Y, м')
plt.legend()
plt.axis([0, 90000, 0, 14000])
plt.grid(True)
plt.show()

for i in range(0, dm):
    plt.plot(ti_[i], aa_[i], color[i], label='dv = ' + str(g_d_2[i]))

plt.xlabel('t, c')
plt.ylabel('alf, град')
plt.legend()
plt.axis([0, 750, -20, 20])
plt.grid(True)
plt.show()

for i in range(0, dm):
    plt.plot(ti_[i], cyy_[i], color[i], label='dv = ' + str(g_d_2[i]))

plt.xlabel('t, c')
plt.ylabel('Cy')
plt.grid(True)
plt.show()

for i in range(0, dm):
    plt.plot(ti_[i], cxx_[i], color[i], label='dv = ' + str(g_d_2[i]))

plt.xlabel('t, c')
plt.ylabel('Cx')
plt.grid(True)
plt.show()

for i in range(0, dm):
    plt.plot(ti_[i], mzz_[i], color[i], label='dv = ' + str(g_d_2[i]))

plt.xlabel('t, c')
plt.ylabel('mz')
plt.grid(True)
plt.show()

