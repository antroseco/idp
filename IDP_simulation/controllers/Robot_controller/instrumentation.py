import logging
from functools import wraps
from typing import Any, Callable, TypeVar

from controller import Robot

AnyCallable = TypeVar('AnyCallable', bound=Callable[..., Any])


def _get_time() -> float:
    """Gets the current simulation time from Webots.

    Returns:
        float: Simulation time in seconds.
    """
    # The Robot.getTime(self) method corresponds to the C API's double wb_robot_get_time which doesn't take any
    # arguments. This suggests that we don't actually need a Robot instance to call this method, so pass None.
    return Robot.getTime(None)


def trace(func: AnyCallable) -> AnyCallable:
    """Decorator used to trace function calls and execution time (simulation time).

    Args:
        func (AnyCallable): Function to trace.

    Returns:
        AnyCallable: Wrapped function.
    """
    @wraps(func)
    def wrapper(*args, **kwargs):
        log = logging.getLogger('tracing')

        start_time = _get_time()

        log.debug(f'{func.__name__} called')
        result = func(*args, **kwargs)
        log.debug(f'{func.__name__} returned after {_get_time() - start_time:.3f} s')

        return result

    return wrapper
