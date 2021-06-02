from time import sleep

class MachineState():
    def __init__(self, initialState="whitout_state", period_time = 4):
        self.state = initialState
        self.period_time = period_time

    def changeState(self, change_state):
        self.state = change_state

state_variable = ["initial_state", "area_state","color_state", "shape_state"]

while True:
    opencv_layer = MachineState()
    for i in state_variable:
        opencv_layer.changeState(i)
        print (opencv_layer.state + "\n")
        sleep(5)
        
    break
print("end of the program")

        


