from scipy.optimize import differential_evolution, minimize, minimize_scalar, shgo

from pytrajlib import simulation, utils

run_params = utils.get_run_params()
run_params["num_runs"] = 1000
run_params.update(
    {
        "initial_x_error": 0,
        "initial_pos_error": 0,
        "initial_vel_error": 0,
        "initial_angle_error": 0,
        "acc_scale_stability": 0,
        "gyro_bias_stability": 0,
        "gyro_noise": 0,
        "gnss_noise": 0,
    }
)


def f(params):
    gearing_ratio, nav_gain = params
    run_params["gearing_ratio"] = gearing_ratio
    run_params["nav_gain"] = nav_gain
    impact_data = simulation.run(config=run_params)
    cep = utils.get_cep(run_params=run_params, impact_data=impact_data)
    print(f"{cep=} {params=}")
    return cep


def f_scalar(gearing_ratio):
    run_params["gearing_ratio"] = gearing_ratio
    impact_data = simulation.run(config=run_params)
    cep = utils.get_cep(run_params=run_params, impact_data=impact_data)
    print(f"{cep=} {gearing_ratio=}")
    return cep


# res = differential_evolution(f_scalar,
#                       bounds=[(1e-2, 1e2)])

res = shgo(f_scalar, bounds=[(1e-2, 1e2)])


# res = minimize(f_scalar,
#                       bounds=[(1e-2, 1e2)], x0=(1))

# res = minimize(
#     f,
#     bounds=[(1e-2, 1e2), (3, 5)],
#     x0=(1, 4),
# )

# res = differential_evolution(
#     f,
#     bounds=[(1e-2, 1e2), (3, 5)],
#     x0=(1, 4),
# )


print(f"Optimal gearing ratio & nav gain: {res.x}, with CEP: {res.fun}")
