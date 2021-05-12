# Manages states
class StateManager:
    def __init__(self, initialState):
        self.state = initialState

    # Sets the state to a certain value
    def changeState(self, newState):
        self.state = newState
        return True

    # Checks if the state corresponds to a specific value
    def checkState(self, state):
        return self.state == state

# Makes it possible to run arbitrary code sequentially without interrupting other code that must run continuoulsy
class SequenceManager:
    def __init__(self):
        self.lineIdentifier = 0
        self.linePointer = 1
        self.done = False

    # Resets the sequence and makes it start from the first event
    def resetSequence(self):
        self.linePointer = 1
        print("----------------")
        print("reseting sequence")
        print("----------------")

    def seqResetSequence(self):
        if self.check():
            self.resetSequence()
            
            return True
        return False

    # This has to be at the start of any sequence of events
    def startSequence(self):
        self.lineIdentifier = 0
        self.done = False

    # Returns if the line pointer and identifier match and increases the identifier
    # Must be included at the end of any sequential function
    def check(self):
        self.done = False
        self.lineIdentifier += 1
        return self.lineIdentifier == self.linePointer

    # Changes to the next event
    def nextSeq(self):
        self.linePointer += 1
        self.done = True

    # returns if the sequence has reached its end
    def seqDone(self):
        return self.done

    # Can be used to make a function sequential or used in an if statement to make a code block sequential
    def simpleSeqEvent(self, function=None, *args, **kwargs):
        if self.check():
            if function is not None:
                function(*args, **kwargs)
            self.nextSeq()
            return True
        return False

    # The function inputted must return True when it ends
    def complexSeqEvent(self, function, *args, **kwargs):
        if self.check():
            if function(*args, **kwargs):
                self.nextSeq()
                return True
        return False
    
    # When inpuuted any function it returns a sequential version of it that can be used in a sequence
    def makeSimpleSeqEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                function(*args, **kwargs)
                self.nextSeq()
                return True
            return False
        return event

    # When inputted a function that returns True when it ends returns a sequential version of it that can be used in a sequence
    def makeComplexSeqEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                if function(*args, **kwargs):
                    self.nextSeq()
                    return True
            return False
        return event