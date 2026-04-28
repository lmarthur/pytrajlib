
def _plot_gyro_error(t, gyro_error_pitch, gyro_error_yaw, save_path):
    """Gyro error (pitch and yaw) vs time in degrees."""
    plt.figure(figsize=(10, 6))
    plt.plot(t, gyro_error_pitch, label="Gyro Error Pitch", linewidth=2)
    plt.plot(t, gyro_error_yaw, label="Gyro Error Yaw", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Gyro Error (degrees)")
    plt.title("Gyro Error vs Time")
    plt.legend(frameon=False)
    plt.grid(alpha=0.3)
    if save_path:
        _save_figure(Path(save_path), "gyro_error.png")
    else:
        plt.show()
    plt.close()