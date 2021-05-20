""" from random import randint
from time import sleep

State = type("State",(object,), {})

class LightsOn():
    def execute():
        print("Lights on!")

class LightsOff():
    def execute():
        print("Lights off!")


class Transitions(object):
    def __init__(self, toState):
        self.toState = toState

    def execute():
        print("Transitioning...")

class SimpleFSM(object):
    def __init__(self, char):
        self.char = char
        self.states = {}
        self.transitions = {}
        self.currentState= None
        self.trans = None 
    
    def SetState(self, stateName):
        self.currentState = self.states[stateName]

    def Transition(self, transName):
        self.trans = self.transitions[transName]

    def execute(self):
        if(self.trans):
            self.trans.execute()
            self.SetState(self.trans.toState)
            self.trans = None
        self.currentState.execute()


class Character(object):
    def __init__(self):
        self.FSM = SimpleFSM(self)
        self.LightOn = True



if __name__ == "__main__":
    light = Character()
    light.FSM.states["On"] = LightsOn()
    light.FSM.states["Off"] = LightsOff()

    light.FSM.transitions["toOn"] = Transitions("On")
    light.FSM.transitions["toOff"] = Transitions("Off")

    light.FSM.SetState("On")

    for i in range(20):
        timeInterval = 1
        startTime = sleep(timeInterval)
        while(timeInterval + timeInterval > sleep(timeInterval)):
            pass
        if(randint(0,2)):
            if(light.LightOn):
                light.FSM.Transition("toOff")
                light.LightOn = False
            else:
                light.FSM.Transition("toOn")
                light.LightOn = True
        light.FSM.execute()    

 """

from time import sleep

class stateMachine():
    def __init__(self):
        self.currentState = "zero"
                
    def onState(self):
        self.currentState = "on State"
        return self.currentState

    def offState(self):
        self.currentState = "off State"

    def onChangeEvent(self):
        if(self.currentState == "zero"):
            self.currentState = self.onState
            #print(f"changin from {self.currentState} to {self.onState}...")
            print("changin state....")
            sleep(6)
            print("done")
        
        elif(self.currentState == "on State"):
            self.currentState = self.offState
        
        elif(self.currentState == "off State"):
            self.currentState == self.onState


""" print("\n\n")
myMachine = stateMachine()
print(myMachine.currentState)
myMachine.offState()
print(myMachine.currentState)
myMachine.onState()
print(myMachine.currentState) """

print("\n\n")
myMachine = stateMachine()
print(f"Initial state --> {myMachine.currentState}")
myMachine.onChangeEvent()
print(f"\n current state is --> {myMachine.currentState}")