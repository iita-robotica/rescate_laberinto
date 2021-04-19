from controller import Robot
import math

timeStep = 16 * 2

class SequenceManager:
    def __init__(self):
        self.lineIdentifier = 0
        self.linePointer = 1
        self.done = False

    def resetSequence(self):
        self.linePointer = 1

    def startSequence(self):
        self.lineIdentifier = 0
        self.done = False

    def check(self):
        self.done = False
        self.lineIdentifier += 1
        return self.lineIdentifier == self.linePointer

    def nextSeq(self):
        self.linePointer += 1
        self.done = True

    def seqDone(self):
        return self.done

    # Can be used to make a function sequential or used in an if statement to make a code block sequential
    def simpleSeqEvent(self, function=None):
        if self.check():
            if function is not None:
                function()
                self.nextSeq()
            return True
        return False

    # The function inputted must return True when it ends
    def complexSeqEvent(self, function):
        if self.check():
            if function():
                self.nextSeq()
                return True
        return False
    
    def makeSimpleSeqEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                function(*args, **kwargs)
                self.nextSeq()
                return True
            return False
        return event

    def makeComplexSeqEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                if function(*args, **kwargs):
                    self.nextSeq()
                    return True
            return False
        return event


robot = Robot()

seqMg = SequenceManager()
seqPrint = seqMg.makeSimpleSeqEvent(print)

while robot.step(timeStep) != -1:
    seqMg.startSequence()
    seqPrint("Hello")
    seqMg.resetSequence()