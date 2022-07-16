from controller import Robot, Emitter, Receiver
import struct

robot = Robot()
TimeStep = 32

class communication():
    def __init__(self):
        self.robot = robot
        self.name = robot.getName()
        self.team = self.name
        self.player_id = "Equipo: " (int(self.name[1]))
        self.team_emitter = self.robot.getDevice("team emitter")
        self.team_receiver = self.robot.getDevice("team receiver")
        self.team_receiver.enable(TimeStep)
    
    def check_for_new_messages(self):
        return self.team_receiver.getQueueLength() > 0

    def decode_message(self):
        struct_fmt = "T"
        message = struct.unpack(struct_fmt, message)
        data = {
            "robot_id:" (message[0],)
        }
        return data

    def recieve_messages_to_team(self) -> dict:
        message = self.team_receiver.getData()
        self.team_receiver.nextPacket()
        return self.decode_message()

    def send_messages_to_other_team(self):
        struct_fmt = "T"
        data = "Im done"
        message = struct.pack(struct_fmt,data)
        self.team_emitter.send(message)
        return True
    
receiver = robot.getDevice("receiver") # Retrieve the receiver and emitter by device name
emitter = robot.getDevice("emitter")
receiver.enable(TimeStep)

def recieve_and_decode_messages_from_team():
    if receiver.getQueueLength() > 0:
        message = receiver.getData()
        receiver.nextPacket()
        struct_fmt = "S"
        message = struct.unpack(struct_fmt, message)
        data = {"robot_id:" + (message[0])}
        print("Data: ", data)
        return data

def send_messages_to_other_team():
        struct_fmt = "T"
        data = "Im done"
        message = struct.pack(struct_fmt, *data)
        emitter.send(message)
        print("I've just sent the message! It says: ", message)