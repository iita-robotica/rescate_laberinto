from flags import SHOW_DEBUG


class DelayManager:
    def __init__(self) -> None:
        self.time = 0
        self.delay_first_time = True
        self.delay_start = 0
    
    def update(self, time):
        self.time = time

    def delay_seconds(self, delay):
            if SHOW_DEBUG:
                print("Current delay: ", delay)
            if self.delay_first_time:
                self.delay_start = self.time
                self.delay_first_time = False
            else:
                if self.time - self.delay_start >= delay:
                    
                    self.delay_first_time = True
                    return True
            return False
    
    def reset_delay(self):
         self.delay_first_time = True