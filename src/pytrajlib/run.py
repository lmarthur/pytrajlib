from tqdm import tqdm

from pytrajlib._traj import ffi
from pytrajlib._traj import lib as traj

_keep_alive = {}


@ffi.def_extern()
def update_loading_bar(n, total):
    """
    Create or update the loading bar with the current progress and total.
    This is called from the C code.
    """
    if _keep_alive.get("loading_bar") is None:
        _keep_alive["loading_bar"] = tqdm(total=total, desc="Progress")
        _keep_alive["loading_bar"].update(n=n)
    else:
        current_progress = _keep_alive["loading_bar"].n
        update_size = n - current_progress
        _keep_alive["loading_bar"].update(n=update_size)
        _keep_alive["loading_bar"].refresh()


def main():
    print("running...")
    N = 100
    r = traj.fly(N)
    results = r.results
    positions = [results[i].impact_event.true_state.position for i in range(N)]
    print(results[0].t, positions[0].x, positions[0].y, positions[0].z)
    print(results[1].t, positions[1].x, positions[1].y, positions[1].z)

    print("finished!")

    _keep_alive = {}


if __name__ == "__main__":
    main()
