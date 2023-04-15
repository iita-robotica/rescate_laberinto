from flags import SHOW_DEBUG



class Sequencer:
    """
    Makes it possible to run arbitrary code sequentially without interrupting other code that must run continuoulsy.
    For example, one can make the robot execute a series of pre-programmed functions with delays and so on, without interrupting
    a sensor that must run continously. 
    This functions basically as an alternative to multithreading or multiprocessing.
    """
    def __init__(self, reset_function=None):
        self.line_identifier = 0
        self.line_pointer = 1
        self.done = False
        self.reset_function = reset_function

    def reset_sequence(self):
        """
        Resets the sequence and makes it start from the first event.
        """
        if self.reset_function is not None:
            self.reset_function()
        self.line_pointer = 1
        if SHOW_DEBUG:
            print("----------------")
            print("reseting sequence")
            print("----------------")

    def seq_reset_sequence(self):
        if self.check():
            self.reset_sequence()
            
            return True
        return False

    def start_sequence(self):
        """
        Starts the sequence. This must be at the start of any sequence of events.
        """
        self.line_identifier = 0
        self.done = False


    def check(self):
        """
        Returns if the line pointer and identifier match and increases the identifier.
        Must be included at the end of any sequential function.
        """
        self.done = False
        self.line_identifier += 1
        return self.line_identifier == self.line_pointer

    def next_seq(self):
        """
        Changes to the next event.
        """
        self.line_pointer += 1
        self.done = True

    def seq_done(self):
        """
        returns if the sequence has reached its end
        """
        return self.done

    def simple_event(self, function=None, *args, **kwargs):
        """
        Can be used to make a function sequential or used with an if statement to make a code block sequential.
        """
        if self.check():
            if function is not None:
                function(*args, **kwargs)
            self.next_seq()
            return True
        return False

    def complex_event(self, function, *args, **kwargs):
        """
        Can be used to make a function sequential. The function inputted must return True when it ends
        """
        if self.check():
            if function(*args, **kwargs):
                self.next_seq()
                return True
        return False
    
    def make_simple_event(self, function):
        """
        When inpuuted any function it returns a sequential version of it that can be used in a sequence.
        """
        def event(*args, **kwargs):
            if self.check():
                function(*args, **kwargs)
                self.next_seq()
                return True
            return False
        return event

    def make_complex_event(self, function):
        """
        When inputted a function that returns True when it ends returns a sequential version of it that can be used in a sequence.
        """
        def event(*args, **kwargs):
            if self.check():
                if function(*args, **kwargs):
                    self.next_seq()
                    return True
            return False
        return event
