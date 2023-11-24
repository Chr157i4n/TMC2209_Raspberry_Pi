#pylint: disable=invalid-name
"""
TMC2209_Raspberry_Pi math library
"""



def rps_to_vactual(rps, steps_per_rev, fclk = 12000000):
    """converts rps -> vactual

    Args:
        rps (float): revolutions per second
        steps_per_rev (int): steps per revolution
        fclk (int): clock speed of the tmc (Default value = 12000000)

    Returns:
        vactual (int): value for vactual
    """
    return int(round(rps / (fclk / 16777216) * steps_per_rev))


def vactual_to_rps(vactual, steps_per_rev, fclk = 12000000):
    """converts vactual -> rps

    Args:
        vactual (int): value for VACTUAL
        steps_per_rev (int): steps per revolution
        fclk (int): clock speed of the tmc (Default value = 12000000)

    Returns:
        rps (float): revolutions per second
    """
    return vactual * (fclk / 16777216) / steps_per_rev


def rps_to_steps(rps, steps_per_rev):
    """converts rps -> steps/second

    Args:
        rps (float): revolutions per second
        steps_per_rev (int): steps per revolution

    Returns:
        steps (int): steps per second
    """
    return rps * steps_per_rev

def steps_to_rps(steps, steps_per_rev):
    """converts steps/second -> rps

    Args:
        steps (int): speed in steps per second
        steps_per_rev (int): steps per revolution

    Returns:
        rps (float): revolutions per second
    """
    return steps / steps_per_rev


def rps_to_tstep(rps, steps_per_rev, msres):
    """converts rps -> tstep

    Args:
        rps (float): revolutions per second
        steps_per_rev (int): steps per revolution
        msres (int): µstep resolution

    Returns:
        tstep (int): time per step
    """
    return int(round(12000000 / (rps_to_steps(rps, steps_per_rev) * 256 / msres)))


def steps_to_tstep(steps, msres):
    """converts steps/second -> tstep

    Args:
        steps (int): speed in steps per second
        msres (int): µstep resolution

    Returns:
        tstep (int): time per step
    """
    return int(round(12000000 / (steps * 256 / msres)))
