
G_TO_MPSS = 9.80665



def compute_height(pressure: float) -> float:
    """
    the conversion uses the formula:
    
    h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
    where:
    h  = height above sea level
    T0 = standard temperature at sea level = 288.15
    L0 = standard temperatur elapse rate = -0.0065
    p  = measured pressure
    P0 = static pressure = 1013.25
    g0 = gravitational acceleration = 9.80665
    M  = mloecular mass of earth's air = 0.0289644
    R* = universal gas constant = 8.31432
    Given the constants, this works out to:
    h = 44330.8 * (1 - (p / P0)**0.190263)
    
    Arguments:
        pressure {float} -- [description]
    
    Returns:
        [type] -- [description]
    """
    return 44330.8 * (1 - pow(pressure / 1013.25, 0.190263))
