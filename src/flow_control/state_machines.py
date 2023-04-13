from flags import SHOW_DEBUG

class StateManager:
    """
    A simple state machine.
    """
    def __init__(self, initialState):
        self.state = initialState

    def changeState(self, newState):
        """Sets the state the specified value."""
        self.state = newState
        return True

    def checkState(self, state):
        """Checks if the state corresponds the specified value."""
        return self.state == state

class SequenceManager:
    """
    Makes it possible to run arbitrary code sequentially without interrupting other code that must run continuoulsy.
    For example, one can make the robot execute a series of pre-programmed functions with delays and so on, without interrupting
    a sensor that must run continously. 
    This functions basically as an alternative to multithreading or multiprocessing.
    """
    def __init__(self, resetFunction=None):
        self.lineIdentifier = 0
        self.linePointer = 1
        self.done = False
        self.resetFunction = resetFunction

    def resetSequence(self):
        """
        Resets the sequence and makes it start from the first event.
        """
        if self.resetFunction is not None:
            self.resetFunction()
        self.linePointer = 1
        if SHOW_DEBUG:
            print("----------------")
            print("reseting sequence")
            print("----------------")

    def seqResetSequence(self):
        if self.check():
            self.resetSequence()
            
            return True
        return False

    def startSequence(self):
        """
        Starts the sequence. This must be at the start of any sequence of events.
        """
        self.lineIdentifier = 0
        self.done = False


    def check(self):
        """
        Returns if the line pointer and identifier match and increases the identifier.
        Must be included at the end of any sequential function.
        """
        self.done = False
        self.lineIdentifier += 1
        return self.lineIdentifier == self.linePointer

    def nextSeq(self):
        """
        Changes to the next event.
        """
        self.linePointer += 1
        self.done = True

    def seqDone(self):
        """
        returns if the sequence has reached its end
        """
        return self.done

    def simpleEvent(self, function=None, *args, **kwargs):
        """
        Can be used to make a function sequential or used with an if statement to make a code block sequential.
        """
        if self.check():
            if function is not None:
                function(*args, **kwargs)
            self.nextSeq()
            return True
        return False

    def complexEvent(self, function, *args, **kwargs):
        """
        Can be used to make a function sequential. The function inputted must return True when it ends
        """
        if self.check():
            if function(*args, **kwargs):
                self.nextSeq()
                return True
        return False
    
    def makeSimpleEvent(self, function):
        """
        When inpuuted any function it returns a sequential version of it that can be used in a sequence.
        """
        def event(*args, **kwargs):
            if self.check():
                function(*args, **kwargs)
                self.nextSeq()
                return True
            return False
        return event

    def makeComplexEvent(self, function):
        """
        When inputted a function that returns True when it ends returns a sequential version of it that can be used in a sequence.
        """
        def event(*args, **kwargs):
            if self.check():
                if function(*args, **kwargs):
                    self.nextSeq()
                    return True
            return False
        return event
