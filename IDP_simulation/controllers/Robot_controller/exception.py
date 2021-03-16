from functools import wraps
from typing import Any, Callable, TypeVar


class CollisionPreventionException(Exception):
    """Exceptions for control flow. Yes, this is bad.
    """
    pass


AnyCallable = TypeVar('AnyCallable', bound=Callable[..., Any])


def reroute_after_collision_prevention(func: AnyCallable) -> AnyCallable:
    """Decorator used to call high level navigation functions again if they
    were interrupted due to collision prevention.

    Args:
        func (AnyCallable): Function to call again.

    Returns:
        AnyCallable: Wrapped function.
    """
    @wraps(func)
    def wrapper(*args, **kwargs):
        # TODO: this probably shouldn't be an infinite loop
        while True:
            try:
                return func(*args, **kwargs)
            except(CollisionPreventionException):
                # retry
                print('REROUTING!')
                pass

    return wrapper
