from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

app = FastAPI()

# Simpan koneksi aktif (Jetson & Browser)
active_connections = set()

# 1. Endpoint WebSocket (Jalur Data)
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.add(websocket)
    print(f"[+] Client Connected. Total: {len(active_connections)}")
    try:
        while True:
            # Terima data (biasanya dari Jetson)
            data = await websocket.receive_text()
            
            # Broadcast ke semua client lain (Browser)
            dead_connections = set()
            for connection in active_connections:
                if connection != websocket:
                    try:
                        await connection.send_text(data)
                    except:
                        dead_connections.add(connection)
            
            # Bersihkan koneksi mati
            for dead in dead_connections:
                active_connections.remove(dead)
                
    except WebSocketDisconnect:
        active_connections.remove(websocket)
        print(f"[-] Client Disconnected. Total: {len(active_connections)}")

# 2. Sajikan file index.html secara otomatis di halaman utama
# Pastikan file 'index.html' ada di folder yang sama dengan script ini
app.mount("/", StaticFiles(directory=".", html=True), name="static")

if __name__ == "__main__":
    # Jalankan server di semua IP pada port 8000
    uvicorn.run(app, host="0.0.0.0", port=8000)