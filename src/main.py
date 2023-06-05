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
    support_values = [-0.13, -0.07, 0.07, 0.13] # The support values for lateral motion
    # Analytic resolution
    cop0, com0, _, _, zk_min0, zk_max0, x0 = simulation_with_feedback(t_step, steps, g, h_com, r_q, xk_init, support_values)
    x0 = x0[:len(cop0)]
    plt.plot(x0, zk_min0[:len(cop0)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x0, zk_max0[:len(cop0)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x0, cop0, color="green", label="cop", linewidth=0.7)
    # plt.scatter(x, cop, s=0.5)
    plt.plot(x0, com0, color="red", label="com", linewidth=1)
    plt.title("Analytic resolution of the problem")
    plt.legend(loc="upper right")
    plt.show()
    # Analytic resolution with perturbation
    cop1, com1, _, _, zk_min1, zk_max1, x1 = simulation_with_perturbations(t_step, steps, g, h_com,
                                                                           r_q, xk_init, inst_perturbation=2.5,
                                                                           acc_perturbation=1)
    x1 = x1[:len(cop1)]
    plt.plot(x1, zk_min1[:len(cop1)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x1, zk_max1[:len(cop1)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x1, cop1, color="green", label="cop", linewidth=0.7)
    # plt.scatter(x, cop, s=0.5)
    plt.plot(x1, com1, color="red", label="com", linewidth=1)
    plt.title("Analytic resolution of the problem with perturbations")
    plt.legend(loc="upper right")
    plt.show()

    # QP resolution

    cop2, com2, _, _, zk_min2, zk_max2, x2 = simulation_qp(t_step, steps, g, h_com, xk_init, support_values)
    x2 = x2[:len(cop2)]
    plt.plot(x2, zk_min2[:len(cop2)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x2, zk_max2[:len(cop2)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x2, cop2, color="green", label="cop", linewidth=0.7)
    # plt.scatter(x, cop, s=0.5)
    plt.plot(x2, com2, color="red", label="com", linewidth=1)
    plt.title("QP resolution of the problem")
    plt.legend(loc="upper right")
    plt.show()

    # QP resolution with perturbations
    cop3, com3, _, _, zk_min3, zk_max3, x3 = simulation_qp_perturbations(t_step, steps, g, h_com,
                                                                           r_q, xk_init, inst_perturbation=2.5,
                                                                           acc_perturbation=1)
    x3 = x3[:len(cop3)]
    plt.plot(x3, zk_min3[:len(cop3)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x3, zk_max3[:len(cop3)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x3, cop3, color="green", label="cop", linewidth=0.7)
    # plt.scatter(x, cop, s=0.5)
    plt.plot(x3, com3, color="red", label="com", linewidth=1)
    plt.title("QP resolution of the problem with perturbations")
    plt.legend(loc="upper right")
    plt.show()




if __name__ == "__main__":
    main()