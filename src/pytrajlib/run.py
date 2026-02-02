# from ._traj import ffi
from pytrajlib._traj import lib as traj


def main():
    print("running...")
    traj.run()
    print("finished!")


if __name__ == "__main__":
    main()
