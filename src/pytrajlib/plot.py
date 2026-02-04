import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

from pytrajlib.utils import (
    Result,
    cartcoords_to_sphercoords,
    cep_from_local_impact,
    get_local_impact,
    haversine_distance,
)


def impact(result: Result, output_dir="results"):
    """
    Plot the impact data from the simulation.
    """
    # Get longitude and latitude of aimpoint and launchpoint
    _, aimpoint_lat, aimpoint_lon = cartcoords_to_sphercoords(result.aimpoint)
    launch_lat, launch_lon = 0, 0

    # Calculate the range to the aimpoint over the surface of the Earth
    # This is the great circle distance between the aimpoint and the origin
    range_to_aimpoint = haversine_distance(
        (launch_lat, launch_lon), (aimpoint_lat, aimpoint_lon)
    )
    print("Range to aimpoint: ", range_to_aimpoint)

    local_impact = get_local_impact(result)
    impact_x_local, impact_y_local = local_impact
    miss_distance, cep = cep_from_local_impact(local_impact)

    print(f"CEP: {cep:.3f}m")
    plotrange = 4 * cep

    # Plot the data
    params = {
        "axes.labelsize": 8,
        "font.size": 8,
        "font.family": "serif",
        "legend.fontsize": 10,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
    }
    plt.rcParams.update(params)

    fig = plt.figure(figsize=(5, 5))
    # plot a circle of radius CEP m centered on (0,0)
    N = 400
    t = np.linspace(0, 2 * np.pi, N)
    x, y = cep * np.cos(t), cep * np.sin(t)

    # gridspec
    gs = fig.add_gridspec(
        2,
        1,
        height_ratios=(6, 1),
        hspace=0.18,
        bottom=0.1,
        top=0.95,
        left=0.025,
        right=0.975,
    )

    a0 = fig.add_subplot(gs[0, 0])
    a1 = fig.add_subplot(gs[1, 0])

    a0.scatter(
        impact_x_local,
        impact_y_local,
        c="grey",
        marker="x",
        label="Impact Points",
        s=20,
        alpha=0.5,
        linewidths=1,
    )
    a0.plot(x, y, c="k", label="CEP", linestyle="--", linewidth=1.5)
    a0.legend(["Impact Points", "CEP"], frameon=False, framealpha=0)

    # center the plot on (0,0)
    a0.set_xlim(-plotrange, plotrange)
    a0.set_ylim(-plotrange, plotrange)
    a0.set_aspect("equal")

    # add N=len(guided_r) to the top left of the plot
    a0.text(
        -0.6 * plotrange,
        0.8 * plotrange,
        f"N = {len(miss_distance)}\nCEP = {cep:.2f}m",
        fontsize=10,
        verticalalignment="top",
        horizontalalignment="center",
    )

    # add label to a0
    a0.set_xlabel("Downrange (m)", labelpad=-1)
    a0.set_ylabel("Crossrange (m)", labelpad=-1)
    a0.tick_params(axis="x", which="major", pad=1)  # Adjust pad for x-axis ticks
    a0.tick_params(axis="y", which="major", pad=1)  # Adjust pad for y-axis ticks

    a0.set_title(result.name)
    # plot the histogram of the miss distances
    # Fit a Nakagami distribution to the data
    x = np.linspace(0, 5 * cep, 100)
    shape, loc, scale = stats.nakagami.fit(miss_distance, floc=0)
    nakagamipdf = stats.nakagami.pdf(x, shape, loc, scale)
    print("Nakagami fit: shape =", shape, "loc =", loc, "scale =", scale)

    # Compute number of bins for the histogram
    bins = 50
    # plot histogram up to 5 times the CEP, with no y axis
    a1.hist(
        miss_distance,
        bins=bins,
        range=(0, 5 * cep),
        color="grey",
        edgecolor="black",
        alpha=0.7,
        histtype="stepfilled",
    )
    # renormalize the pdfs to the histogram
    nakagamipdf = nakagamipdf * len(miss_distance) * 5 * cep / bins
    # evaluate the pdf at the CEP
    pdf_cep = nakagamipdf[np.argmin(np.abs(x - cep))]
    # Add a vertical line at the CEP, to the top of the histogram at that point
    plotmax = a1.get_ylim()[1]
    a1.axvline(
        x=cep,
        ymax=pdf_cep / plotmax,
        color="k",
        linestyle="--",
        linewidth=1.5,
        label="CEP",
    )

    a1.plot(
        x,
        nakagamipdf,
        "k",
        linewidth=1.5,
        label="Nakagami(" + str(round(shape, 2)) + ", " + str(round(scale, 2)) + ")",
    )

    # omit the frame
    a1.spines["top"].set_visible(False)
    a1.spines["right"].set_visible(False)
    a1.spines["left"].set_visible(False)
    a1.spines["bottom"].set_visible(False)

    a1.yaxis.set_visible(False)
    # Legend with no border or box around it, and with the nakagami fit parameters
    a1.legend(frameon=False, framealpha=0)
    a1.tick_params(axis="x", which="major", pad=1)
    a1.tick_params(axis="y", which="major", pad=1)
    a1.set_xlabel("Miss Distance Histogram (m)", labelpad=1)

    if output_dir is not None:
        plt.savefig(output_dir + "/impact_plot.jpg", dpi=1000)
        plt.savefig(output_dir + "/impact_plot.pdf")
        plt.close()
    return cep
