from functools import wraps
from time import time


def timeit(func):
    @wraps(func)
    def timed_func(*args, **kwargs):
        t = time()
        res = func(*args, **kwargs)
        print(f'{func.__name__} ran in {time() - t}.')
        return res

    return timed_func


from collections import defaultdict


class FuncTimer:
    def __init__(self):
        self._total_times = defaultdict(lambda: (0, 0))

    def time(self, func):
        @wraps(func)
        def timed_func(*args, **kwargs):
            t = time()
            res = func(*args, **kwargs)
            exec_time, count = self._total_times[func.__name__]
            self._total_times[func.__name__] = (exec_time + time() - t, count + 1)
            return res

        return timed_func

    def refresh(self):
        self._total_times = defaultdict(lambda: (0, 0))

    def report(self):
        for name, (exec_time, count) in self._total_times.items():
            print(f'{name} ran a total of {count} times in {exec_time:.2f}s, average : {exec_time / count}s')