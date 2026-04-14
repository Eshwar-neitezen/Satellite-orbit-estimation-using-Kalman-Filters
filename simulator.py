import numpy as np
from scipy.integrate import solve_ivp
from dynamics import dynamics


def simulate_orbit(initial_state=None,t_span=(0, 6000),num_steps=1000):

    if initial_state is None:
        initial_state = [7000e3, 0, 0, 0, 7500, 0]

    t_eval = np.linspace(t_span[0], t_span[1], num_steps)

    solution = solve_ivp(
        fun=dynamics,
        t_span=t_span,
        y0=initial_state,
        t_eval=t_eval,
        method="RK45",
        rtol=1e-9,
        atol=1e-9
    )

    if not solution.success:
        raise RuntimeError(f"Integration failed: {solution.message}")

    t = solution.t          # shape: (N,)
    state = solution.y      # shape: (6, N)

    return t, state


if __name__ == "__main__":
    t, state = simulate_orbit()
    print(f"Simulation complete.")
    print(f"Time steps : {t.shape[0]}")
    print(f"State shape: {state.shape}")
    print(f"Final position (m): {state[:3, -1]}")
    print(f"Final velocity (m/s): {state[3:, -1]}")