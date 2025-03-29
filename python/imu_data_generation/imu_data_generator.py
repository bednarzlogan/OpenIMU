import matplotlib.backend_bases
import matplotlib.pyplot as plt
import matplotlib
from typing import Any, Dict, List, Tuple


def small_dynamics(position_setpoints: List[Tuple[float, float]], 
                   vehicle_constraints: Dict[str, float]) -> List[Any]:
    state_data: List[Any] = []

    # we'll implement a linear bicycle model
        # r_k = r_{k-1} + v_{k-1}*dt
        # v_k = v_{k+1} + 

    return state_data


def main() -> int:
    """
    Brief: 
        This script is used to generate IMU data based on spacial derivative arguments.
        This is not preferred to real IMU data, rather, it's a stepping stone for pure
        simulation testing.
    """
    # TODO:
    #   Small GUI for clicking points to make trajectory
    #   Define a standard size for the grid that you're clicking on
    #   Define the max movement in one sample interval
    #   Perform simple math operations to determine PVA

    # allocate a list for clicked points
    clicked_points: List[Tuple[float, float]] = []

    # create grid
    plot_struct: Tuple[plt.figure, plt.axes] = plt.subplots()
    fig, ax = plot_struct
    
    # set bounds
    # TODO - set sizing requirements programatically
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_title("Click on the grid to set positions")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.grid(True)

    def onclick(event: matplotlib.backend_bases.MouseEvent)->None:
        # Check if the click is within the axes
        if event.inaxes == ax:
            x, y = event.xdata, event.ydata
            clicked_points.append((x, y))
            print(f"Clicked at: x={x:.2f}, y={y:.2f}")
            # Mark the clicked point on the plot
            ax.plot(x, y, 'ro')  # 'ro' means red circle
            plt.draw()

    # Connect the click event to the handler function
    _ = fig.canvas.mpl_connect('button_press_event', onclick)

    plt.show()

    return 1


if __name__ == "__main__":
    ret: int = main()
    print(f"Program exited with code {ret}")