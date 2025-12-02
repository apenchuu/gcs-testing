import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
from std_msgs.msg import String, UInt8
from core.utils.config import Topic, PxMode, Param
from core_msgs.msg import Pixhawk

LAPTOP_URI = "ws://100.78.180.2:8000/ws"

class Gcs(Node):
    def __init__(self, loop, node = Node):
        super().__init__('Gcs')
        self.node = node
        self.loop = loop 
        
        self.ws_connection = None   
        self.loop.create_task(self.connect_to_server())

        self.lon_history = []
        self.lat_history = []
        self.pxmode = PxMode.HOLD


        Param.TRACK.createParam(self.node, default_value="B")
        self.track = Param.TRACK.getValue(self.node) 
        self.track_pub = Topic.arena.createPublisher(self.node)
        self.track_pub.publish(String(data=self.track))

        self.image_subscriber = Topic.camera_processed.createSubscriber(
            self,
            self.image_callback
        )
        self.blue_box_subscriber = Topic.image_blue_box.createSubscriber(
            self,
            self.blue_box_callback
        )
        self.green_box_subscriber = Topic.image_green_box.createSubscriber(
            self,
            self.green_box_callback
        )
        self.pixhawk_subscriber = Topic.pixhawk.createSubscriber(
            self,
            self.pixhawk_callback
        )
        self.mission_subscriber = Topic.mission.createSubscriber(
            self,
            self.mission_callback
        )
        self.pxmode_subscriber = Topic.pxmode.createSubscriber(
            self,
            self.pxmode_callback
        )
        
    
    async def connect_to_server(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Trying to Connect {LAPTOP_URI}...")
                # Connect ke Laptop
                async with websockets.connect(LAPTOP_URI) as websocket:
                    self.ws_connection = websocket
                    self.get_logger().info("Connected")
                    
                    # Tunggu sampai koneksi putus (block di sini)
                    await websocket.wait_closed()
            except Exception as e:
                self.get_logger().warn(f"Failed to Connect: {e}")
            
            # Jika putus/gagal, reset variabel dan tunggu sebelum reconnect
            self.ws_connection = None
            self.get_logger().info("Reconnecting...")
            await asyncio.sleep(3)


    def pxmode_callback(self, msg: String):
        self.pxmode = msg.data

    def image_callback(self, msg: String):
        self._handle_incoming_data("camera_processed", msg.data)

    def blue_box_callback(self, msg: String):
        self._handle_incoming_data("image_blue_box", msg.data)

    def green_box_callback(self, msg: String):
        self._handle_incoming_data("image_green_box", msg.data)

    def mission_callback(self, msg: UInt8):
        self._handle_incoming_data("mission", msg.data)

    def pixhawk_callback(self, msg: Pixhawk):
        if(msg.lat > 1):
            return

        if self.pxmode == PxMode.HOLD:
            self.lon_history = [msg.lon]
            self.lat_history = [msg.lat]
        else:
            self.lon_history.append(msg.lon)
            self.lat_history.append(msg.lat)

        data = {
            "lon": self.lon_history,
            "lat": self.lat_history,
            "alt": msg.alt,
            "msg_spd": msg.msg_spd,
            "msg_heading": msg.msg_heading,
            "track": self.track,
        }
        self._handle_incoming_data("pixhawk", data)

    def _handle_incoming_data(self, topic_name, data):
        message = {"topic": topic_name, "data": data}
        # self.get_logger().info(f"[{topic_name}] Received: {data}")

        asyncio.run_coroutine_threadsafe(
            self.broadcast_message(message),
            self.loop
        )
    
    async def broadcast_message(self, message):
        if self.ws_connection:
            try:
                await self.ws_connection.send(json.dumps({"data": message}))
            except Exception as e:
                self.get_logger().warn(f"Failed sending data: {e}")

async def main_async():
    rclpy.init()
    loop = asyncio.get_running_loop() 
    node = rclpy.create_node('gcs_node')

    gcs = Gcs(loop, node)

    # Vibe coding research later
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(gcs)

    # Jalankan spin di thread terpisah agar loop asyncio tidak terblokir
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        while rclpy.ok():
            await asyncio.sleep(1)
    finally:
        gcs.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()