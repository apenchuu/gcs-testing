# File: simple_ros_sender.py (Jalankan di Jetson)
import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json

# --- GANTI IP INI DENGAN IP TAILSCALE LAPTOP ANDA ---
SERVER_URI = "ws://100.78.180.2:8000" 

class SimpleSender(Node):
    def __init__(self, loop):
        super().__init__('simple_sender')
        self.loop = loop
        self.ws_connection = None
        self.counter = 0
        
        # Buat Timer: Jalan setiap 1 detik
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Mulai koneksi background
        self.loop.create_task(self.connect_to_server())

    async def connect_to_server(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Mencoba connect ke {SERVER_URI}...")
                async with websockets.connect(SERVER_URI) as websocket:
                    self.ws_connection = websocket
                    self.get_logger().info("Terhubung!")
                    await websocket.wait_closed()
            except Exception as e:
                self.get_logger().warn("Gagal connect, mencoba lagi...")
            
            self.ws_connection = None
            await asyncio.sleep(2)

    def timer_callback(self):
        # 1. Buat Dummy Data
        self.counter += 1
        data_dummy = {
            "pesan": "Halo dari Jetson",
            "angka": self.counter,
            "status": "Aman"
        }
        
        # 2. Kirim ke Server Laptop
        self.get_logger().info(f"Mengirim data ke-{self.counter}")
        
        if self.ws_connection and not self.ws_connection.closed:
            # Kirim secara async tanpa memblokir ROS
            asyncio.run_coroutine_threadsafe(
                self.ws_connection.send(json.dumps(data_dummy)),
                self.loop
            )

async def main_async():
    rclpy.init()
    loop = asyncio.get_running_loop()
    
    node = SimpleSender(loop)
    
    # Jalankan ROS di thread terpisah agar tidak mengganggu WebSocket
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    loop.run_in_executor(None, executor.spin)
    
    try:
        while rclpy.ok():
            await asyncio.sleep(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()