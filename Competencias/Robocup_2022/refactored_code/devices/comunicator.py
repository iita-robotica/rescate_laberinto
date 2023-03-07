import utilities
import struct

class Comunicator:
    def __init__(self, emmiter, receiver, timeStep):
        self.receiver = receiver
        self.emmiter = emmiter
        self.receiver.enable(timeStep)
        self.lackOfProgress = False
        self.doGetWordInfo = True
        self.gameScore = 0
        self.remainingTime = 0

    def sendVictim(self, position, victimtype):
        self.doGetWordInfo = False
        letter = bytes(victimtype, "utf-8")
        position = utilities.multiplyLists(position, [100, 100])
        position = [int(position[0]), int(position[1])]
        message = struct.pack("i i c", position[0], position[1], letter)
        self.emmiter.send(message)
        self.doGetWordInfo = False

    def sendLackOfProgress(self):
        self.doGetWordInfo = False
        message = struct.pack('c', 'L'.encode())  # message = 'L' to activate lack of progress
        self.emmiter.send(message)
        self.doGetWordInfo = False

    def sendEndOfPlay(self):
        self.doGetWordInfo = False
        exit_mes = struct.pack('c', b'E')
        self.emmiter.send(exit_mes)
        print("Ended!!!!!")

    def sendMap(self, npArray):
        # Get shape
        print(npArray)
        s = npArray.shape
        # Get shape as bytes
        s_bytes = struct.pack('2i', *s)
        # Flattening the matrix and join with ','
        flatMap = ','.join(npArray.flatten())
        # Encode
        sub_bytes = flatMap.encode('utf-8')
        # Add togeather, shape + map
        a_bytes = s_bytes + sub_bytes
        # Send map data
        self.emmiter.send(a_bytes)
        # STEP3 Send map evaluate request
        map_evaluate_request = struct.pack('c', b'M')
        self.emmiter.send(map_evaluate_request)
        self.doGetWordInfo = False

    def requestGameData(self):
        if self.doGetWordInfo:
            message = struct.pack('c', 'G'.encode())  # message = 'G' for game information
            self.emmiter.send(message)  # send message

    def update(self):

        if self.doGetWordInfo:
            self.requestGameData()
            if self.receiver.getQueueLength() > 0: # If receiver queue is not empty
                receivedData = self.receiver.getBytes()
                if len(receivedData) > 2:
                    tup = struct.unpack('c f i', receivedData) # Parse data into char, float, int
                    if tup[0].decode("utf-8") == 'G':
                        self.gameScore = tup[1]
                        self.remainingTime = tup[2]
                        self.receiver.nextPacket() # Discard the current data packet

            self.lackOfProgress = False
            if self.receiver.getQueueLength() > 0:  # If receiver queue is not empty
                receivedData = self.receiver.getBytes()
                print(receivedData)
                if len(receivedData) < 2:
                    tup = struct.unpack('c', receivedData)  # Parse data into character
                    if tup[0].decode("utf-8") == 'L':  # 'L' means lack of progress occurred
                        print("Detected Lack of Progress!")
                        self.lackOfProgress = True
                    self.receiver.nextPacket()  # Discard the current data packetelse:
        else:
            self.doGetWordInfo = True