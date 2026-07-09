import marimo

__generated_with = "0.23.9"
app = marimo.App()


@app.cell
def _():
    import marimo as mo

    import pytrajlib as ptl

    return mo, ptl


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    Access the default configuration and modify vehicle specifications:
    """)
    return


@app.cell
def _(ptl):
    custom_config = ptl.get_default_config()
    custom_config["vehicle"]["booster"]["stages"][0]["burn_time"] = 30
    custom_config
    return (custom_config,)


@app.cell
def _(custom_config, ptl):
    ptl.run(
        config=custom_config,
        num_runs=30,
        plot_impact=True,
        plot_trajectory=True,
        output_dir=None,
    )
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    If you do want detailed trajectory data, you can access the data frames and produce the plots yourself:
    """)
    return


@app.cell
def _(ptl):
    impact_df, config, trajectory_df, guidance_df = ptl.run(
        num_runs=30, return_config=True, return_trajectory=True, return_guidance=True
    )
    aimpoint = (config["x_aim"], config["y_aim"], config["z_aim"])
    return aimpoint, guidance_df, impact_df, trajectory_df


@app.cell
def _(impact_df):
    impact_df
    return


@app.cell
def _(guidance_df):
    guidance_df
    return


@app.cell
def _(aimpoint, impact_df, ptl):
    ptl.create_impact_plot(impact_df=impact_df, aimpoint=aimpoint)
    return


@app.cell
def _(aimpoint, guidance_df, ptl, trajectory_df):
    ptl.create_traj_plots(
        trajectory_df=trajectory_df, aimpoint=aimpoint, guidance_df=guidance_df
    )
    return


if __name__ == "__main__":
    app.run()
