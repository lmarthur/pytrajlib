import pandas as pd


def parse_impact_results_to_dfs(results, N):
    columns = ["t", "x", "y", "z"]
    df = pd.DataFrame(columns=columns)

    times = [results[i].t for i in range(N)]
    df.t = times

    positions = [results[i].impact_event.position for i in range(N)]
    df.x = [positions[i].x for i in range(N)]
    df.y = [positions[i].y for i in range(N)]
    df.z = [positions[i].z for i in range(N)]

    return df
