from math import radians


REPLICA_DECOUPLING_ANGLE = radians(80)
STATUETTE_DECOUPLING_ANGLE = radians(5)


class WebotsStatuetteDriver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        timestep = int(self.__robot.getBasicTimeStep())
        self.__connector = self.__robot.getDevice('hand_L_statuette_connector')
        self.__connector.enablePresence(timestep)
        self.__motor = self.__robot.getDevice('hand_mid_L')
        self.__encoder = self.__motor.getPositionSensor()
        self.__encoder.enable(timestep)
        self.__finished = False
        self.__enable_statuette_decoupling = False

    def step(self):
        """
        Check in every time step if statuette should be decoupled.

        In order to decouple the statuette, we first need to decouple the
        replica. We have a three state FSM:
        coupled_statuette -> decoupled_replica -> decoupled_statuette.
        """
        if self.__finished:
            return
        if self.__connector.getPresence() and not self.__connector.isLocked():
            # state: coupled_statuette
            self.__connector.lock()
        elif self.__encoder.getValue() > REPLICA_DECOUPLING_ANGLE:
            self.__enable_statuette_decoupling = True
        elif self.__encoder.getValue(
        ) <= STATUETTE_DECOUPLING_ANGLE and self.__connector.isLocked(
        ) and self.__enable_statuette_decoupling:
            self.__connector.unlock()
            self.__finished = True
