import time

# Clase de capa de obstarccion
# Abstraction layer class

class AbstractionLayer:
    def __init__(self, initialState=""):
        self.state = initialState
        self.startTime = self.actualTime = time.time()
        self.lineIdentifier = -1
        self.linePointer = 0
        self.delayStart = 0.0
        self.delayFirstTime = True
    
    
    # Poner antes de empezar una sequencia o de usar una funcion sequencial
    # Put before starting a sequence or using a sequencial function
    def startSequence(self):
        self.lineIdentifier = -1
    
    # Para la sequencia por la cantidad de segundos que uno le ponga
    # Stops a sequence for the given amount of seconds 
    def seqDelay(self, delay):
        self.lineIdentifier += 1
        if self.lineIdentifier == self.linePointer:
            if self.delayFirstTime:
                self.delayStart = time.time()
                self.delayFirstTime = False
            else:
                if self.actualTime - self.delayStart >= delay:
                    self.delayFirstTime = True
                    self.linePointer += 1
                    return True
        

    # Hace un print en sequencia
    # Prints something in sequence
    def seqPrint(self, text):
        self.lineIdentifier += 1
        if self.lineIdentifier == self.linePointer:
            print(text)
            self.linePointer += 1
            return True
        

    # Cambia el estado
    # Changes the state
    def changeState(self, newState):
        self.state = newState
        self.linePointer = 0

    # Poner al inicio del loop principal
    # Put at the start of the main loop
    def update(self):
        self.actualTime = time.time()

# Instanciacion de capa de abstracción
# Abstraction layer instantiation
r = AbstractionLayer("start")

while True:
    r.update()

    if r.state == "start":
        #Esto corre cada timestep
        #This runs every timestep
        print("start state!")
        r.changeState("main")
    
    elif r.state == "main":
        #Esto corre cada timestep
        #This runs every timestep
        print("main state!")
        #Esto corre en sequencia
        #This runs in sequence
        r.startSequence()
        r.seqPrint("Antes de seqDelay / before seqDelay")
        r.seqDelay(0.1)
        if r.seqPrint("Despues de seqDelay / after seqDelay"):
            r.changeState("overTime")

    elif r.state == "overTime":
        #Esto corre cada timestep
        #This runs every timestep
        print("overtime!")
        r.changeState("end")
    
    elif r.state == "end":
        break

    