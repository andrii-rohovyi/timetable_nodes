def to_milliseconds(td: float) -> int:
    """
    Converts timedelta to milliseconds and round the value.

    Parameters
    ----------
    td: float
        Time delta in seconds.
    Returns
    -------
    int
        Time delta in milliseconds.
    """
    return int(td * 1000)
