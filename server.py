# File: simple_server.py (Jalankan di Laptop)
import asyncio
import websockets

async def handler(websocket):
    print(f"--> Client Connected dari: {websocket.remote_address}")
    try:
        async for message in websocket:
            print(f"Data Masuk: {message}")
    except websockets.ConnectionClosed:
        print("--> Client Disconnected")

async def main():
    # 0.0.0.0 artinya mendengarkan dari semua IP (termasuk Tailscale)
    print("Server siap menerima data di Port 8000...")
    async with websockets.serve(handler, "0.0.0.0", 8000):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())