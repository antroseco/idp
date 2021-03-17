from functools import wraps
from typing import Any, Callable, TypeVar


class CollisionPreventionException(Exception):
    """Exceptions for control flow. Yes, this is bad.
    """
    pass


AnyCallable = TypeVar('AnyCallable', bound=Callable[..., Any])


def reroute_after_collision_prevention(robot):
    """Decorator used to call high level navigation functions again if they
    were interrupted due to collision prevention.

    Args:
        func (AnyCallable): Function to call again.

    Returns:
        AnyCallable: Wrapped function.
    """
    def decorator_factory(func: AnyCallable) -> AnyCallable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            reroute = kwargs.pop('reroute', True)

            if reroute:
                robot.throw_on_collision_prevention = True

            # TODO: this probably shouldn't be an infinite loop
            while True:
                try:
                    result = func(*args, **kwargs)

                    if reroute:
                        robot.throw_on_collision_prevention = False

                    return result
                except CollisionPreventionException as e:
                    if not reroute:
                        raise e
                    # else retry

        return wrapper

    return decorator_factory
