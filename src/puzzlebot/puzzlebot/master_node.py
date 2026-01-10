#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from collections import deque
import time

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')

        # Debounce de línea
        self.line_buffer = deque(maxlen=15)
        self.line_lost = False
        self.line_threshold = 10

        # FSM
        self.state = 'SEEK_LINE'
        self.signal = 0
        self.semaphore = 0
        self.pending_action = None
        self.last_state_change = time.time()
        self.line_lost_start_time = None
        self.action_timeout = 4.0
        self.curve_phase = 0
        self.curve_start_time = None

        # Suscripciones
        self.create_subscription(Int32, '/signal_detected', self.signal_callback, 10)
        self.create_subscription(Int32, '/light_state_int', self.semaphore_callback, 10)
        self.create_subscription(Bool, '/line_lost', self.raw_line_callback, 10)

        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.priority_pub = self.create_publisher(Bool, '/control_priority', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.state_machine)

    def signal_callback(self, msg):
        # Guardar intención para cruce/curva si no hay pendiente
        if self.pending_action is None and msg.data in [2,3,4]:
            self.pending_action = 'CROSSING' if msg.data==2 else 'TURNING'
            self.get_logger().info(f"📝 Acción pendiente guardada: {self.pending_action}")
        self.signal = msg.data

    def semaphore_callback(self, msg):
        self.semaphore = msg.data

    def raw_line_callback(self, msg):
        # Debounce de línea
        self.line_buffer.append(msg.data)
        lost_count = sum(self.line_buffer)
        new_lost = lost_count >= self.line_threshold
        if new_lost != self.line_lost:
            self.line_lost = new_lost
            estado = 'PERDIDA' if new_lost else 'RECUPERADA'
            self.get_logger().info(f"📶 Línea {estado} ({lost_count}/{len(self.line_buffer)})")
            if not new_lost:
                self.line_lost_start_time = None

    def state_machine(self):
        now = time.time()
        self.get_logger().info(f"🔍 Estado: {self.state} | Pend: {self.pending_action} | Sem: {self.semaphore} | LíneaLost: {self.line_lost}")

        if self.state == 'SEEK_LINE':
            self.publish_priority(False)

            # STOP tiene prioridad absoluta
            if self.signal == 1:
                self.pending_action = None
                self.transition('STOPPED')
                return

            # Grace period si línea breve perdida y sin pendiente
            if self.line_lost and not self.pending_action:
                if self.line_lost_start_time is None:
                    self.line_lost_start_time = now
                elif now - self.line_lost_start_time < 2.0:
                    self.drive_forward(speed=0.1)
                    return
                else:
                    self.transition('LINE_LOST')
                    return

            if self.line_lost:
                # Curva inmediata sin semáforo
                if self.pending_action == 'TURNING':
                    self.transition('TURNING')
                # Cruce sólo con semáforo verde
                elif self.pending_action == 'CROSSING' and self.semaphore==3:
                    self.transition('CROSSING')
                # Si semáforo no verde y hay pendiente
                elif self.semaphore!=3 and self.pending_action:
                    self.transition('STOPPED')
                else:
                    # Señales normales
                    if self.signal==2 and self.semaphore==3:
                        self.transition('CROSSING')
                    elif self.signal in [3,4]:
                        self.transition('TURNING')
                    else:
                        self.transition('LINE_LOST')

        elif self.state == 'STOPPED':
            self.publish_priority(True)
            # Reanudar curva sin semáforo, cruce con semáforo verde
            if self.pending_action == 'TURNING':
                self.transition('TURNING')
            elif self.pending_action == 'CROSSING' and self.semaphore==3:
                self.transition('CROSSING')
            elif now - self.last_state_change > 3.0:
                self.transition('SEEK_LINE')
            else:
                self.stop_robot()

        elif self.state == 'TURNING':
            self.publish_priority(True)
            if self.curve_phase==0:
                self.curve_start_time=now; self.curve_phase=1; self.drive_forward(speed=0.15)
            elif self.curve_phase==1 and now-self.curve_start_time>=1.0:
                self.curve_start_time=now; self.curve_phase=2
            elif self.curve_phase==2:
                if not self.line_lost:
                    self.transition('SEEK_LINE')
                elif now-self.curve_start_time<=1.0:
                    self.perform_right_curve()
                else:
                    self.curve_start_time=now; self.curve_phase=3
            elif self.curve_phase==3:
                if now-self.curve_start_time<=0.15:
                    self.drive_forward(speed=0.1)
                else:
                    self.transition('SEEK_LINE')

        elif self.state == 'CROSSING':
            self.publish_priority(True)
            if self.line_lost_start_time is None:
                self.line_lost_start_time=now
            if not self.line_lost:
                self.transition('SEEK_LINE')
            elif now-self.line_lost_start_time < self.action_timeout:
                self.drive_forward()
            else:
                self.transition('LINE_LOST')

        elif self.state == 'LINE_LOST':
            self.publish_priority(True)
            if not self.line_lost:
                self.transition('SEEK_LINE')
                self.pending_action=None
            else:
                self.explore_motion()

    def transition(self, new_state):
        self.get_logger().info(f"🔄 Transición: {self.state} → {new_state}")
        self.state=new_state
        self.last_state_change=time.time()
        self.line_lost_start_time=None
        self.curve_phase=0
        self.curve_start_time=None
        if new_state == 'SEEK_LINE':
            self.pending_action = None

    def perform_right_curve(self):
        twist=Twist(); twist.linear.x=0.05; twist.angular.z=-0.18; self.cmd_pub.publish(twist)
    def drive_forward(self,speed=0.1):
        twist=Twist(); twist.linear.x=speed; twist.angular.z=0.0; self.cmd_pub.publish(twist)
    def explore_motion(self):
        twist=Twist(); twist.linear.x=0.05; twist.angular.z=0.15; self.cmd_pub.publish(twist)
    def stop_robot(self):
        twist=Twist(); twist.linear.x=0.0; twist.angular.z=0.0; self.cmd_pub.publish(twist)
    def publish_priority(self,master_has_control):
        self.priority_pub.publish(Bool(data=master_has_control))

def main(args=None):
    rclpy.init(args=args)
    node=MasterNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
