import argparse

def run_simulation(config):
    print("Reading configuration file " + config + "...")
    print("running simulation")

def cli():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c", "--config", type=str, required=True, help="Path to the configuration file."
    )

    # Run the simulation
    run_simulation(**vars(parser.parse_args()))