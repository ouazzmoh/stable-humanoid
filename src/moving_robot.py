from utils import *
from simulations import *
import matplotlib.pyplot as plt




def main():
    t_step = 5e-3
    # We simulate from 0 to 9 (s)
    steps = int(9 / t_step)
    g = 9.81
    h_com = 0.8
    r_q = 1e-6
    xk_init = (0, 0, 0)
    yk_init = (0, 0, 0)

    support_values_forward = [-0.13, -0.01, 0.01, 0.13]
    support_values_lat = [-0.13, -0.07, 0.07, 0.13]


    zk_min, zk_max = construct_zmin_zmax_moving(steps=steps, duration_double_init=0.03,
                                                duration_step=0.07, duration_transition=0.018, foot_size=0.13,
                                                spacing=0.14)

    plt.plot(zk_min)
    plt.plot(zk_max)
    plt.plot((zk_min + zk_max)/2)
    plt.show()

    # zk_min_lat, zk_max_lat = construct_zmin_zmax(steps=steps, duration_double_init=0.26, duration_left=0.07,
    #                                      duration_right=0.07, duration_transition=0.018,
    #                                      min_val_left=support_values_lat[0], max_val_left=support_values_lat[1],
    #                                      min_val_right=support_values_lat[2], max_val_right=support_values_lat[3])


    # cop_lateral, com_lateral, _, _, zk_min_lat, zk_max_lat, x = simulation_qp(t_step, steps, g, h_com, xk_init,
    #                                                                   zk_min_lat, zk_max_lat)
    # cop_forward, com_forward, _, _, zk_min_forward, zk_max_forward, x = simulation_qp(t_step, steps, g, h_com, xk_init,
    #                                                                                   zk_min_forward, zk_max_forward)




if __name__ == "__main__":
    main()

