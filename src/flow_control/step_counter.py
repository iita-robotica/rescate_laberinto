class StepCounter:
    def __init__(self, interval):
        self.__current_step = 0
        self.interval = interval

    def increase(self):
        self.__current_step += 1
        if self.__current_step == self.interval:
            self.__current_step = 0
    
    def check(self):
        return self.__current_step == 0