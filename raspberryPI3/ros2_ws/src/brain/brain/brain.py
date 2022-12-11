import rclpy
from rclpy.node import Node
from interfaces.msg import Package
from interfaces.msg import Travelling
from statemachine import StateMachine, State
import time

'''
STATE MACHINE POSTCAR

|-----------------------------------------------------------------|        Blocked<-------|
|                                                                 |                       |
Idle --> Travelling to source --> Waiting package --> Travelling to destination --> Waiting fetch
| ^                                      |                                                |
| |<-------------------------------------|------------------------------------------------|
↓ | 
Mapping 
'''
class Postcar(StateMachine):
    # States
    idle = State('Idle', initial=True)
    travelling_source = State('Travelling to source')
    waiting_package = State('Waiting package')
    travelling_dest = State('Travelling to destination')
    waiting_fetch = State('Waiting Fetch')
    mapping = State('Mapping')
    blocked = State('Blocked')

    # Transitions
    start_mapping = idle.to(mapping)
    stop_mapping = mapping.to(mapping)
    travel_source = idle.to(travelling_source)
    arrrived_source = travelling_source.to(waiting_package)
    timeout_wait = waiting_package.to(idle)
    got_package = waiting_package.to(travelling_dest)
    lost_package = travelling_dest.to(idle) # lost package when travelling to destination
    arrived_dest = travelling_dest.to(waiting_fetch)
    timeout_fetch = waiting_fetch.to(blocked)
    package_fetched = waiting_fetch.to(idle)

class Brain(Node):

    #State Machine
    postcar = Postcar()
    timenow = time()

    def __init__(self):
        super().__init__('brain')

        # Variables
        self.package = Package()
        self.travelling = Travelling()

        # Subscriptions
        self.subscription_package = self.create_subscription(Package, 'detect_package', self.updateState_package, 10)
        self.subscription_travelling = self.create_subscription(Travelling, 'travel', self.updateState_travel, 10)

    def updateState_package(self):
        if self.postcar.current_state == Postcar.waiting_package and self.subscription_package.state_pack == True:
            self.postcar.got_package()
        elif time()-self.timenow > 600.0:
            #TO DO: alert user that fetching failed
            self.postcar.timeout_wait()

    def updateState_travel(self):
        if self.postcar.current_state == Postcar.idle and self.subscription_travelling.travelling == True:
            self.postcar.travel_source()
        elif self.postcar.current_state == Postcar.travel_source and self.subscription_travelling.travelling == False:
            self.postcar.arrrived_source()
        elif self.postcar.current_state == Postcar.travelling_dest and self.subscription_travelling.travelling == False:



def main():
    rclpy.init()

    brain = Brain()
    rclpy.spin(brain)
    brain.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()