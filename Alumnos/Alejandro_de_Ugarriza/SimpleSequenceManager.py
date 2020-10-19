import time

class AbstractionLayer:
    def __init__(self, initialState=""):
        self.state = initialState
        self.startTime = self.actualTime = time.time()
        self.identifier = self.linePointer = 0
        self.delayStart = time.time()
        self.delayFirstTime = True
    
    def changeState(self, newState):
        if self.identifier == self.linePointer:
            self.state = newState
            self.linePointer = 0
        self.identifier += 1

    def startSequence(self):
        self.identifier = 0
    
    
    def delay(self, delay):
        if self.identifier == self.linePointer:
            if self.delayFirstTime:
                self.delayStart = time.time()
                self.delayFirstTime = False
            else:
                if self.actualTime - self.delayStart >= delay:
                    self.delayFirstTime = True
                    self.linePointer += 1
        self.identifier += 1


    def update(self):
        self.actualTime = time.time()

r = AbstractionLayer("start")

while True:
    r.update()

    if r.state == "start":
        #Esto corre cada timestep
        #This runs every timestep
        print("start state!")
        #Esto corre en sequencia
        #This runs in sequence
        r.startSequence()
        r.changeState("main")
    
    elif r.state == "main":
        #Esto corre cada timestep
        #This runs every timestep
        print("main state!")
        #Esto corre en sequencia
        #This runs in sequence
        r.startSequence()
        r.delay(0.1)
        r.changeState("overTime")

    elif r.state == "overTime":
        #Esto corre cada timestep
        #This runs every timestep
        print("overtime!")
        #Esto corre en sequencia
        #This runs in sequence
        r.startSequence()
        r.changeState("end")

    