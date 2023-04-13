class StepCounter:
    """
    Allows to execute actions every n number of timesteps. This can be useful for performance, as it enables the program
    to execute taxing tasks sparsely while not interrupting actions that must run constantly.
    """

    def __init__(self, interval):
        self.__current_step = 0
        self.interval = interval

    def increase(self):
        self.__current_step += 1
        if self.__current_step == self.interval:
            self.__current_step = 0
    
    def check(self):
        return self.__current_step == 0