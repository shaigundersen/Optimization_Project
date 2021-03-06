import sys
import numpy as np
import matplotlib.pyplot as plt
from math import pi
from pyomo.environ import *

def epsilon_method(model, solver, f2_min, f2_max):
    """
    Function for finding Pareto frontier using the epsilon constraint method

    :param model: predefined model initialized with two basic vars (x,y), their boundaries, basic constraints
                  and two objectives (f1, f2)
    :param solver: optimization solver used to solve this model
    :param f2_min: anchor value of f2
    :param f2_max: anchor value of f2
    :return: tuple of x,y and f1,f2 values for Pareto frontier
    """
    # create new objective  -> min f1
    # create new constraint -> f2 <= epsilon
    model.new_obj = Objective(expr=model.f1, sense=minimize)
    model.e = Param(initialize=0, mutable=True)
    model.c_temp = Constraint(expr=model.f2 <= model.e)

    # create range of 10 epsilons
    n = 10
    step = (f2_max - f2_min) / n
    epsilons = np.arange(f2_min, f2_max, step)
    x_list, y_list = [], []
    f1_list, f2_list = [], []
    for ep in epsilons:
        model.e = ep
        solver.solve(model)
        x_list.append(value(model.x))
        y_list.append(value(model.y))
        f1_list.append(value(model.new_obj))
        f2_list.append(value(model.c_temp))

    # delete all temp components
    model.del_component(model.c_temp)
    model.del_component(model.e)
    model.del_component(model.new_obj)
    return x_list, y_list, f1_list, f2_list, "Epsilon Constraint Method"

def plot(**params):
    X_1, Y_1, F1_1, F2_1, title_1 = params['subplot1']
    X_2, Y_2, F1_2, F2_2, title_2 = params['subplot2']

    fig, axs = plt.subplots(nrows=2, ncols=2, figsize=(20, 10))
    ax1, ax2, ax3, ax4 = axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1]

    edge, = ax1.plot(X_1, Y_1, 'o', markersize=6, markerfacecolor='none', c='b')
    shaded = ax1.fill_between(X_1, Y_1, color='azure', alpha=0.85)
    ax1.legend([(edge, shaded)], ['Decision/coordinate space'], loc='best')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.grid(True)
    ax1.set_title(title_1)
    ax1.set_ylim(bottom=min(Y_1)-0.1*min(Y_1))

    ax2.plot(F1_1, F2_1, 'o-', c='r', label='Pareto optimal front')
    ax2.legend(loc='best')
    ax2.set_xlabel('Objective function F1')
    ax2.set_ylabel('Objective function F2')
    ax2.grid(True)
    ax2.set_title(title_1)

    edge, = ax3.plot(X_2, Y_2, 'o', markersize=6, markerfacecolor='none', c='b')
    shaded = ax3.fill_between(X_2, Y_2, color='azure', alpha=0.85)
    ax3.legend([(edge, shaded)], ['Decision/coordinate space'], loc='best')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.grid(True)
    ax3.set_title(title_2)
    ax3.set_ylim(bottom=min(Y_2)-0.1*min(Y_2))

    ax4.plot(F1_2, F2_2, 'o-', c='r', label='Pareto optimal front')
    ax4.legend(loc='best')
    ax4.set_xlabel('Objective function F1')
    ax4.set_ylabel('Objective function F2')
    ax4.grid(True)
    ax4.set_title(title_2)

    plt.show()

def weighted_sum(model, solver):
    """
    Function for finding Pareto frontier using the weighted sum method

    :param model: predefined model initialized with two basic vars (x,y), their boundaries, basic constraints
                  and two objectives (f1, f2)
    :param solver: optimization solver used to solve this model
    :return: tuple of x,y and f1,f2 values for Pareto frontier
    """
    # create temp components needed for this function
    model.alfa = Param(initialize=0, mutable=True)
    # create new weighted objective -> min alfa*f1 + (1-alfa)*f2
    model.f = Objective(expr=model.alfa * model.f1 + (1 - model.alfa) * model.f2, sense=minimize)
    alfas = np.arange(0, 1, 0.1)
    x_list, y_list = [], []
    f1_list, f2_list = [], []
    for alfa in alfas:
        model.alfa = alfa
        solver.solve(model)
        x_list.append(value(model.x))
        y_list.append(value(model.y))
        f1_list.append(value(model.f1))
        f2_list.append(value(model.f2))

    # delete all temp components
    model.del_component(model.f)
    model.del_component(model.alfa)

    return x_list, y_list, f1_list, f2_list, "Weighted Sum Method"


def get_pareto_min_max_values(model, solver):
    """
    Util function to find 'Anchor' points. Anchor points describe the extremes of the Pareto frontier.
    Those points can be obtained when we minimize one objective while ignoring others.

    :param model: predefined model initialized with two basic vars (x,y), their boundaries, basic constraints
                  and two objectives (f1, f2)
    :param solver: optimization solver used to solve this model
    :return: Anchor values from f2 of Pareto frontier
    """
    # create temp components needed for this function
    model.O_f1 = Objective(expr=model.f1, sense=minimize)
    model.O_f2 = Objective(expr=model.f2, sense=minimize)

    # min f1 separately
    model.O_f1.activate()
    model.O_f2.deactivate()
    solver.solve(model)
    f2_min = value(model.f2)

    # min f2 separately
    model.O_f2.activate()
    model.O_f1.deactivate()
    solver = SolverFactory(solvername, executable=solverpath_exe)
    solver.solve(model)
    f2_max = value(model.f2)

    # delete all temp components
    model.del_component(model.O_f1)
    model.del_component(model.O_f2)

    # get Anchor values from f2 of Pareto frontier
    f2_min, f2_max = min(f2_min, f2_max), max(f2_min, f2_max)
    return f2_min, f2_max

if __name__ == '__main__':
    if len(sys.argv) != 3:
        quit("please provide solvername & solverpath_exe as sys arguments")
    if not sys.argv[2].endswith('.exe'):
        quit("please provide path to solver.exe as second argument")
    # download solver baron and get it's path to exe file
    solvername = sys.argv[1]
    solverpath_exe = sys.argv[2]
    try:
        solver = SolverFactory(solvername, executable=solverpath_exe)

        # Define our model according to the Cones problem #
        model = ConcreteModel()
        # create 2 vars for the problem
        model.x = Var(domain=NonNegativeReals, bounds=(0, 10))
        model.y = Var(domain=NonNegativeReals, bounds=(0, 20))
        # expression to hold slant height
        model.s = Expression(expr=sqrt((model.x**2) + (model.y**2)))  # s = sqrt(r^2 + h^2)
        # create all constrains on vars
        model.c = Constraint(expr=(pi/3)*(model.x**2)*model.y >= 200)
        # define objective function as expressions for later use and manipulating
        model.f1 = Expression(expr=pi * model.x * (model.x + model.s))
        model.f2 = Expression(expr=pi * model.x * model.s)

        # Solve model using two methods
        f_min, f_max = get_pareto_min_max_values(model, solver)
        eps_param = epsilon_method(model, solver, f_min, f_max)
        weighted_param = weighted_sum(model, solver)
        # plot our results
        plot(subplot1=eps_param, subplot2=weighted_param)
    except Exception as e:
        quit(f"An Error accrued\n{e}")
