import time
import logging
from typing import Union


def _check_running_time(start_time: float, duration: float, algo: str) -> Union[str, None]:
    """
    Check it execution time of the process exceeds the allowed duration.

    Parameters
    ----------
    start_time: float
        Time when a process which is checking started.
        Should be `time.monotonic()`.
    duration: float
        Maximal duration of process running, in seconds.
    algo: str
        Name of an algorithm running time of which we're measuring.

    Returns
    -------
    str
        A message notifying that execution time exceeded the allowed duration or None.
    """
    if duration is not None and time.monotonic() - start_time > duration:
        message = f"The duration of {algo} path finding algorithm exceeded " + \
                  f"the max time ({duration} sec)"
        logging.warning(message)
        return message
