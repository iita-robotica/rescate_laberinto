import time

# Clase de capa de obstarccion
# Abstraction layer class
class AbstractionLayer:
    def __init__(self, initialState=""):
        self.state = initialState
        self.startTime = time.time()
        self.actualTime = time.time()
    
    # Cambia el estado
    # Changes the state
    def changeState(self, newState):
        self.state = newState

    # Poner al inicio del loop principal
    # Put at the start of the main loop
    def update(self):
        self.actualTime = time.time()

# Instanciacion de capa de abstracción
# Abstraction layer instantiation
r = AbstractionLayer("start")

while True:
    if r.state == "start":
        print("start state!")
        r.changeState("main")
    
    elif r.state == "main":
        print("main state!")
        if r.actualTime - r.startTime > 0.1:
            r.changeState("overTime")

    elif r.state == "overTime":
        print("overtime!")
        r.changeState("end")

    r.update()