from scipy.interpolate import CubicSpline, make_interp_spline, interp1d
import matplotlib.pyplot as plt
import numpy as np
import functools
from typing import Union


class Interpolate:
    """Trajectory interpolation utility class."""

    @staticmethod
    def time_clip(start, end, interval, unit="s", end_control=False):
        """Generate time sequence with millisecond precision.

        Args:
            start (float): Start time
            end (float): End time
            interval (float): Time interval
            unit (str): Time unit, either "s" or "ms"
            end_control (bool): Whether to control end time based on equality
                between last interpolated value and original last value

        Returns:
            np.ndarray: Time sequence array
        """
        if unit == "s":
            precision = 0.001
        elif unit == "ms":
            precision = 1
        time_line = (np.array([start, end, interval]) / precision).astype("int32")
        time_clipped = np.arange(time_line[0], time_line[1], step=time_line[2])
        if end_control:
            if time_clipped[-1] != time_line[1]:
                time_clipped = np.append(time_clipped, time_line[1])
        else:
            time_clipped = np.append(time_clipped, time_line[1])
        time_clipped = time_clipped.astype("float64")
        time_clipped *= precision
        return time_clipped

    @classmethod
    def no_interpolate(cls, y, t, t_i, unit="s", sort=False, plot=False):
        """No interpolation (step function).

        Args:
            y (np.ndarray): Function values
            t (np.ndarray): Time sequence
            t_i (float or np.ndarray): Interpolation time interval or time sequence
            unit (str): Time unit, either "s" or "ms"
            sort (bool): Whether to sort time sequence to ensure monotonic increase
            plot (bool): Whether to plot the curve

        Returns:
            tuple or list: If t_i is interval, returns (time_sequence, interpolated_values);
                          If t_i is time sequence, returns interpolated_values only
        """
        # Sort time sequence to ensure monotonic increase
        if sort:
            idx = np.argsort(t)
            t, y = t[idx], y[idx]
        # Extend interpolated time sequence
        if isinstance(t_i, (float, int)):
            flag = True
            t_interp = cls.time_clip(t[0], t[-1], t_i, unit)
        else:
            flag = False
            t_interp = t_i
        # No interpolation
        stage_num = len(t)

        def stage(t_):
            for i in range(stage_num - 1):
                if t[i] <= t_ <= t[i + 1]:
                    return y[i + 1]

        y_interp = []
        for t_ in t_interp:
            y_interp.append(stage(t_))
        # Plot curve if requested
        if plot:
            cls.plot(t, y, t_interp, y_interp)
        # Return interpolated time sequence and function values
        if flag:
            return t_interp, y_interp
        else:
            return y_interp

    @classmethod
    def linear_interpolate(cls, y, t, t_i, unit="s", sort=False, plot=False):
        """Linear interpolation.

        Args:
            y (np.ndarray): Function values
            t (np.ndarray): Time sequence
            t_i (float or np.ndarray): Interpolation time interval or time sequence
            unit (str): Time unit, either "s" or "ms"
            sort (bool): Whether to sort time sequence to ensure monotonic increase
            plot (bool): Whether to plot the curve

        Returns:
            tuple or list: If t_i is interval, returns (time_sequence, interpolated_values);
                          If t_i is time sequence, returns interpolated_values only
        """
        # Sort time sequence to ensure monotonic increase
        if sort:
            idx = np.argsort(t)
            t, y = t[idx], y[idx]
        # Extend interpolated time sequence
        if isinstance(t_i, (float, int)):
            flag = True
            t_interp = cls.time_clip(t[0], t[-1], t_i, unit)
        else:
            flag = False
            t_interp = t_i
        # Use LinearNDInterpolator for linear interpolation
        y_interp = interp1d(t, y)(t_interp)
        if plot:
            cls.plot(t, y, t_interp, y_interp)
        # Return interpolated time sequence and function values
        if flag:
            return t_interp, y_interp
        else:
            return y_interp

    @classmethod
    def cubic_spline(
        cls, y, t, t_i, unit="s", bc_type="clamped", sort=False, plot=False
    ):
        """Cubic spline interpolation using CubicSpline.

        Args:
            y (np.ndarray): Function values
            t (np.ndarray): Time sequence (1D array)
            t_i (float or np.ndarray): Interpolation time interval or time sequence
            unit (str): Time unit, either "s" or "ms"
            bc_type (str): Boundary condition type: 'not-a-knot', 'natural', 'clamped'
            sort (bool): Whether to sort time sequence to ensure monotonic increase
            plot (bool): Whether to plot the curve

        Returns:
            tuple or list: If t_i is interval, returns (time_sequence, interpolated_values);
                          If t_i is time sequence, returns interpolated_values only
        """
        # Sort time sequence to ensure monotonic increase
        if sort:
            idx = np.argsort(t)
            t, y = t[idx], y[idx]
        # Extend interpolated time sequence
        if isinstance(t_i, (float, int)):
            flag = True
            t_interp = cls.time_clip(t[0], t[-1], t_i, unit)
        else:
            flag = False
            t_interp = t_i
        # Use CubicSpline for cubic spline interpolation
        cnt = 1
        while True:
            try:
                y_interp = CubicSpline(t, y, bc_type=bc_type)(t_interp)
            except Exception:
                # Handle duplicates or non-strictly increasing sequence
                t[-cnt] += 0.0003 / cnt
            else:
                break
            cnt += 1
        if plot:
            cls.plot(t, y, t_interp, y_interp)
        # Return interpolated time sequence and function values
        if flag:
            return t_interp, y_interp
        else:
            return y_interp

    @classmethod
    def spline(cls, y, t: np.ndarray, t_i, k=5, unit="s", sort=False, plot=False):
        """K-th order spline interpolation using make_interp_spline.

        Commonly used orders are 3rd and 5th order splines.

        Args:
            y (np.ndarray): Function values
            t (np.ndarray): Time sequence (1D array)
            t_i (float or np.ndarray): Interpolation time interval or time sequence
            k (int): Spline order (1-5), commonly 3 or 5
            unit (str): Time unit, either "s" or "ms"
            sort (bool): Whether to sort time sequence to ensure monotonic increase
            plot (bool): Whether to plot the curve

        Returns:
            tuple or list: If t_i is interval, returns (time_sequence, interpolated_values);
                          If t_i is time sequence, returns interpolated_values only
        """
        # Sort time sequence to ensure monotonic increase
        if sort:
            idx = np.argsort(t)
            t, y = t[idx], y[idx]
        # Extend interpolated time sequence
        if isinstance(t_i, (float, int)):  # Specified frequency
            flag = True
            t_interp = cls.time_clip(t[0], t[-1], t_i, unit)
        else:  # Specified refined time points
            flag = False
            t_interp = t_i
        # Use make_interp_spline for spline interpolation
        cnt = 1
        while True:
            try:
                y_interp = make_interp_spline(
                    t, y, k=k, bc_type=(((1, 0), (2, 0)), ((1, 0), (2, 0)))
                )(t_interp)
            except:  # noqa: E722
                # Handle duplicates or non-strictly increasing sequence
                t[-cnt] += 0.0003 / cnt
            else:
                break
            cnt += 1
        if plot:
            cls.plot(t, y, t_interp, y_interp, block=True)
        # Return interpolated time sequence and function values
        if flag:
            return t_interp, y_interp
        else:
            return y_interp

    @staticmethod
    def linear_speed_calculate(y: np.ndarray, t):
        """Calculate linear rate of change between curve points.

        Args:
            y (np.ndarray): Function values
            t (np.ndarray or float): Time sequence or time interval

        Returns:
            np.ndarray: Rate of change values
        """
        y_cp = y.copy()
        y1 = np.delete(y, 0)
        y2 = np.delete(y_cp, -1)
        if isinstance(t, (np.ndarray, list, tuple)):
            return (y1 - y2) / (t[1:] - t[:-1])
        else:
            return (y1 - y2) / t

    @staticmethod
    def interval_limit(y_i: np.ndarray, min_delta=0.001, max_delta=3):
        """Control the interval in y direction after interpolation.

        Args:
            y_i (np.ndarray): Interpolated y values
            min_delta (float): Minimum allowed delta
            max_delta (float): Maximum allowed delta

        Returns:
            np.ndarray: Limited y values
        """
        end_index = y_i.shape[0] - 1
        i = 1
        # Limit middle elements
        while i < end_index:
            if np.fabs(y_i[i] - y_i[i - 1]) < min_delta:
                y_i = np.delete(y_i, i)
                end_index -= 1  # Effective length -1 after deleting middle element
                y_i = np.append(y_i, y_i[end_index])
            else:
                i += 1  # Element count +1 when no deletion
        # Limit last two (original) elements
        if np.fabs(y_i[end_index] - y_i[end_index - 1]) < min_delta:
            y_i[end_index - 1] = y_i[end_index]
        return y_i

    @classmethod
    def way_points_interpolate(
        cls,
        way_points: Union[np.ndarray, tuple],
        time_points: np.ndarray,
        freq: float,
        k=5,
    ) -> list:
        """Waypoint interpolation.

        Args:
            way_points (np.ndarray): Waypoint array
            time_points (np.ndarray): Time points array
            freq (float): Interpolation frequency
            k (int): Spline order

        Returns:
            list: Interpolated joint matrix
        """
        if not isinstance(way_points, np.ndarray):
            way_points = np.vstack(way_points)
        execute_time_array = cls.time_clip(0, time_points[-1], 1 / freq)
        interpolate = functools.partial(
            Interpolate.spline, t=time_points, t_i=execute_time_array, k=k
        )
        joints_matrix_new: np.ndarray = np.apply_along_axis(
            interpolate, axis=0, arr=way_points
        )
        return joints_matrix_new.tolist()

    @staticmethod
    def plot(t, y, t_interp, y_interp, pause=0, clear=True, ion=False, block=False):
        """Plot spline interpolation results.

        Args:
            t (np.ndarray): Original time sequence (1D array)
            y (np.ndarray): Original function values (1D array)
            t_interp (np.ndarray): Interpolated time sequence (1D array)
            y_interp (np.ndarray): Interpolated function values (1D array)
            pause (float): Delay time after display (effective when block=False)
            clear (bool): Whether current plot overwrites previous plot
            ion (bool): Whether to enable interactive mode
            block (bool): Whether to block after displaying plot

        Note:
            Default is "non-blocking + no wait + overwrite + non-interactive"
            for real-time display of latest interpolation results.
        """
        if ion:
            plt.ion()  # Enable interactive mode
        if clear:
            plt.clf()
        # Plot original data points
        plt.plot(t, y, "ro", label="original", markersize=3)
        # Plot spline interpolation results
        plt.plot(t_interp, y_interp, "g-", label="interpolated", markersize=3)
        # Add legend and labels
        plt.legend(loc="best")
        plt.xlabel("t")
        plt.ylabel("y")
        plt.title("Interpolation Result")
        # Show plot
        plt.show(
            block=block
        )  # Non-block doesn't clear previous plot, waits for next plot to overwrite (requires plt.clf())
        if pause > 0 and not block:
            plt.pause(pause)


if __name__ == "__main__":
    # # Example usage
    # t = np.array([0, 2])
    # y = np.array([0, 2])
    # t_i = 2 / 1000
    # Interpolate.spline(y, t, t_i, plot=True)

    # Example usage with way points
    way_points = np.array([[0, 1], [2, 3]])
    print(way_points)
    way_points = ([0, 1], [2, 3])
    time_points = np.array([0, 3])
    freq = 1000
    k = 5
    interpolated_waypoints = Interpolate.way_points_interpolate(
        way_points, time_points, freq, k
    )
    print(interpolated_waypoints[0])
    print(interpolated_waypoints[-1])
