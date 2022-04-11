from math import radians


REPLICA_DECOUPLING_ANGLE = 80   # degrees


class WebotsReplicaDriver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        timestep = int(self.__robot.getBasicTimeStep())
        self.__connector = self.__robot.getDevice('hand_L_replica_connector')
        self.__motor = self.__robot.getDevice('hand_mid_L')
        self.__encoder = self.__motor.getPositionSensor()
        self.__encoder.enable(timestep)
        self.__finished = False

    def step(self):
        """
        Check in every time step if replica should be decoupled.

        If the angle of the motor is above a threshold,
        decouple the replica.
        The threshold is currently set at 80 degrees.
        """
        if self.__finished:
            return
        if self.__encoder.getValue() > radians(REPLICA_DECOUPLING_ANGLE):
            self.__connector.unlock()
            self.__finished = True
