# class taht can only have a single instance. It is used to "communicate"
# battery tresholds between long_term_patroller.py (which dynamically
# reconfigures the tresholds and uses them for point selection) and
# monitor_states.py(which uses them for the battery monitoring)


class ParameterStore(object):
    _instance = None

    def __init__(self):
        self.BATTERY_LOW = 0
        self.BATTERY_VERY_LOW = 0
        self.BATTERY_CHARGED = 0

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(ParameterStore, cls).__new__(
                cls, *args, **kwargs)

        return cls._instance
