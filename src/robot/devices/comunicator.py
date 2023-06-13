import utilities
import struct

from robot.devices.sensor import Sensor

from flags import SHOW_MAP_AT_END

class Comunicator(Sensor):
    def __init__(self, emmiter, receiver, timeStep):
        self.receiver = receiver
        self.emmiter = emmiter
        self.receiver.enable(timeStep)
        self.lack_of_progress = False
        self.do_get_world_info = True
        self.game_score = 0
        self.remaining_time = 0

    def send_victim(self, position, victimtype):
        self.do_get_world_info = False
        letter = bytes(victimtype, "utf-8")
        position = utilities.multiplyLists(position, [100, 100])
        position = [int(position[0]), int(position[1])]
        message = struct.pack("i i c", position[0], position[1], letter)
        self.emmiter.send(message)
        self.do_get_world_info = False

    def send_lack_of_progress(self):
        self.do_get_world_info = False
        message = struct.pack('c', 'L'.encode())  # message = 'L' to activate lack of progress
        self.emmiter.send(message)
        self.do_get_world_info = False

    def send_end_of_play(self):
        self.do_get_world_info = False
        exit_mes = struct.pack('c', b'E')
        self.emmiter.send(exit_mes)
        print("Ended!!!!!")

    def send_map(self, np_array):
        # Get shape
        if SHOW_MAP_AT_END:
            print(np_array)
        s = np_array.shape
        # Get shape as bytes
        s_bytes = struct.pack('2i', *s)
        # Flattening the matrix and join with ','
        flatMap = ','.join(np_array.flatten())
        # Encode
        sub_bytes = flatMap.encode('utf-8')
        # Add togeather, shape + map
        a_bytes = s_bytes + sub_bytes
        # Send map data
        self.emmiter.send(a_bytes)
        # STEP3 Send map evaluate request
        map_evaluate_request = struct.pack('c', b'M')
        self.emmiter.send(map_evaluate_request)
        self.do_get_world_info = False

    def request_game_data(self):
        if self.do_get_world_info:
            message = struct.pack('c', 'G'.encode())  # message = 'G' for game information
            self.emmiter.send(message)  # send message

    def update(self):
        if self.do_get_world_info:
            self.request_game_data()
            if self.receiver.getQueueLength() > 0: # If receiver queue is not empty
                received_data = self.receiver.getBytes()
                if len(received_data) > 2:
                    tup = struct.unpack('c f i', received_data) # Parse data into char, float, int
                    if tup[0].decode("utf-8") == 'G':
                        self.game_score = tup[1]
                        self.remaining_time = tup[2]
                        self.receiver.nextPacket() # Discard the current data packet

            self.lack_of_progress = False
            if self.receiver.getQueueLength() > 0:  # If receiver queue is not empty
                received_data = self.receiver.getBytes()
                print(received_data)
                if len(received_data) < 2:
                    tup = struct.unpack('c', received_data)  # Parse data into character
                    if tup[0].decode("utf-8") == 'L':  # 'L' means lack of progress occurred
                        print("Detected Lack of Progress!")
                        self.lack_of_progress = True
                    self.receiver.nextPacket()  # Discard the current data packetelse:
        else:
            self.do_get_world_info = True