"""
TMC2209_Raspberry_Pi math library
"""



def rps_to_vactual(rps, steps_per_rev:int, fclk:int = 12000000) -> int:
    """converts rps -> vactual

    Args:
        rps (float): revolutions per second
        steps_per_rev (int): steps per revolution
        fclk (int): clock speed of the tmc (Default value = 12000000)

    Returns:
        vactual (int): value for vactual
    """
    return int(round(rps / (fclk / 16777216) * steps_per_rev))


def vactual_to_rps(vactual:int, steps_per_rev:int, fclk:int = 12000000) -> float:
    """converts vactual -> rps

    Args:
        vactual (int): value for VACTUAL
        steps_per_rev (int): steps per revolution
        fclk (int): clock speed of the tmc (Default value = 12000000)

    Returns:
        rps (float): revolutions per second
    """
    return vactual * (fclk / 16777216) / steps_per_rev


def rps_to_steps(rps:float, steps_per_rev:int) -> int:
    """converts rps -> steps/second

    Args:
        rps (float): revolutions per second
        steps_per_rev (int): steps per revolution

    Returns:
        steps (int): steps per second
    """
    return int(round(rps * steps_per_rev))

def steps_to_rps(steps:int, steps_per_rev:int) -> float:
    """converts steps/second -> rps

    Args:
        steps (int): speed in steps per second
        steps_per_rev (int): steps per revolution

    Returns:
        rps (float): revolutions per second
    """
    return steps / steps_per_rev


def rps_to_tstep(rps:float, steps_per_rev:int, mres:int) -> int:
    """converts rps -> tstep

    Args:
        rps (float): revolutions per second
        steps_per_rev (int): steps per revolution
        mres (int): µstep resolution

    Returns:
        tstep (int): time per step
    """
    return int(round(12000000 / (rps_to_steps(rps, steps_per_rev) * 256 / mres)))


def steps_to_tstep(steps:int, mres:int) -> int:
    """converts steps/second -> tstep

    Args:
        steps (int): speed in steps per second
        mres (int): µstep resolution

    Returns:
        tstep (int): time per step
    """
    return int(round(12000000 / (steps * 256 / mres)))


def constrain(val:int, min_val:int, max_val:int) -> int:
    """constrains a value between a min and a max

    Args:
        val (int): value that should be constrained
        min_val (int): minimum value
        max_val (int): maximum value

    Returns:
        int: constrained value
    """
    if val < min_val:
        return min_val
    if val > max_val:
        return max_val
    return val
