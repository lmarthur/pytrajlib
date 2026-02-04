from tqdm import tqdm

from pytrajlib._traj import ffi
from pytrajlib._traj import lib as traj
from pytrajlib.utils import parse_impact_results_to_dfs

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
    N = 1
    r = traj.fly(N)
    results = r.results

    true_df = parse_impact_results_to_dfs(results, N)

    true_df.to_csv("results/impact_states.csv", index=False)

    print("finished!")

    _keep_alive = {}


if __name__ == "__main__":
    main()
