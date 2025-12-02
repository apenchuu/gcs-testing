import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
import math
import random

# GANTI DENGAN IP SERVER LAPTOP (Tailscale/Lokal)
SERVER_URI = "ws://100.78.180.2:8000/ws" 

class SimpleSender(Node):
    def __init__(self, loop):
        super().__init__('simple_sender')
        self.loop = loop
        self.ws_connection = None
        self.counter = 0
        
        # Timer: Kirim data setiap 0.5 detik (2 Hz)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Task untuk koneksi background
        self.loop.create_task(self.connect_to_server())

    async def connect_to_server(self):
        """Menangani koneksi dan auto-reconnect ke Server"""
        while rclpy.ok():
            try:
                self.get_logger().info(f"🔄 Mencoba connect ke {SERVER_URI}...")
                async with websockets.connect(SERVER_URI) as websocket:
                    self.ws_connection = websocket
                    self.get_logger().info("✅ Terhubung ke Server!")
                    await websocket.wait_closed()
            except Exception as e:
                self.get_logger().warn(f"⚠️ Gagal connect: {e}")
            
            self.ws_connection = None
            self.get_logger().info("⏳ Mencoba reconnect dalam 3 detik...")
            await asyncio.sleep(3)

    def send_mock_message(self, topic_name, payload):
        """
        Fungsi helper untuk membungkus data sesuai format Gcs asli:
        { "data": { "topic": "...", "data": ... } }
        """
        if self.ws_connection:
            # Format persis seperti class Gcs asli
            message = {
                "topic": topic_name,
                "data": payload
            }
            final_packet = {"data": message}

            try:
                asyncio.run_coroutine_threadsafe(
                    self.ws_connection.send(json.dumps(final_packet)),
                    self.loop
                )
            except Exception as e:
                self.get_logger().warn(f"Gagal kirim {topic_name}: {e}")

    def timer_callback(self):
        self.counter += 1
        
        # --- 1. MOCK DATA PIXHAWK (Bergerak) ---
        # Membuat koordinat yang bergerak perlahan (simulasi perahu jalan)
        base_lat = -6.362483
        base_lon = 106.824965
        
        # Bergerak melingkar kecil
        offset_lat = math.sin(self.counter * 0.1) * 0.0005
        offset_lon = math.cos(self.counter * 0.1) * 0.0005

        pixhawk_data = {
            "lon": [base_lon + offset_lon], # Format array sesuai history asli
            "lat": [base_lat + offset_lat],
            "alt": round(10.0 + random.uniform(-0.5, 0.5), 2),
            "msg_spd": round(random.uniform(2.0, 5.0), 1),
            "msg_heading": (self.counter * 10) % 360, # Berputar 0-360
            "track": "A" if self.counter % 20 < 10 else "B"
        }
        self.send_mock_message("pixhawk", pixhawk_data)

        # --- 2. MOCK DATA MISSION ---
        mission_data = (self.counter // 10) % 5 # Berubah tiap 10 tick
        self.send_mock_message("mission", mission_data)

        # --- 3. MOCK CAMERA / BOXES ---
        # Kirim data random kadang ada, kadang tidak
        if self.counter % 5 == 0:
            self.send_mock_message("camera_processed", "Person Detected")
            self.send_mock_message("image_blue_box", "Blue Box Center")
        elif self.counter % 5 == 2:
            self.send_mock_message("image_green_box", "Green Box Found")

        self.get_logger().info(f"📤 Data batch #{self.counter} terkirim")

async def main_async():
    rclpy.init()
    loop = asyncio.get_running_loop()
    
    node = SimpleSender(loop)
    
    # Executor untuk menjalankan ROS node
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    # Jalankan spin di thread terpisah agar loop asyncio tidak terblokir
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        # Loop utama asyncio agar program tidak exit
        while rclpy.ok():
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()