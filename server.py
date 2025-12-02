from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn

app = FastAPI()

# Simpan koneksi aktif (Jetson & Browser)
active_connections = set()

# 1. Endpoint WebSocket (Jalur Data)
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.add(websocket)
    print(f"[+] Client Connected (IP: {websocket.client.host}). Total: {len(active_connections)}")
    try:
        while True:
            # Terima data (biasanya dari Jetson)
            data = await websocket.receive_text()
            
            # Broadcast ke semua client lain (Website Next.js)
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

# --- PERUBAHAN: HAPUS BAGIAN app.mount ---
# Kita tidak menyajikan index.html lagi dari sini.
# Tampilan UI sekarang dihandle oleh Next.js di port 3000.

if __name__ == "__main__":
    # Host 0.0.0.0 Wajib agar bisa diakses via Tailscale
    print("🚀 GCS Backend berjalan di Port 8000 (WebSocket Only)")
    uvicorn.run(app, host="0.0.0.0", port=8000)