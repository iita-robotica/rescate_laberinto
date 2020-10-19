import time

class AbstractionLayer:
    def __init__(self, initialState=""):
        self.state = initialState
        self.startTime = time.time()
        self.actualTime = time.time()
    
    def changeState(self, newState):
        self.state = newState

    def update(self):
        self.actualTime = time.time()

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