# 🚀 WebRTC Camera Stream

This project establishes a WebRTC connection between a Jetson-powered rover and a base station, allowing real-time video streaming from the rover's camera to a web-based client application.

---

## 📊 Architecture

```
+--------------------+          HTTP POST /offer           +--------------------+
|                    | <---------------------------------- |                    |
|     Client (UI)    |                                    |     Server (Jetson)|
|  React + WebRTC    | ----------------------------------> |   aiohttp + aiortc |
|                    |           SDP Answer (JSON)         |                    |
+--------------------+                                    +--------------------+
        |                                                            |
        |                   WebRTC Media Stream                     |
        | <========================================================> |
        |                                                            |
```

* **Server (`webrtc.py`)**
  Runs on the Jetson device or rover. Captures live video from a camera (`/dev/video{id}`) and serves it over a WebRTC connection using `aiortc`.

* **Client (React app)**
  Runs at the base station. Initiates a WebRTC connection by sending an SDP offer to the server, receives the video stream, and displays it in the browser.

---

## 🔄 Call Flow

1. **Client sends HTTP POST** to the `/offer` endpoint on the server with an SDP offer.
2. **Server responds with an SDP answer**, setting up a WebRTC connection.
3. **Video media stream** is transmitted from the server to the client via WebRTC.
4. (Optional) **Data channel** can be added for control signaling in the future.

---

## ⚙️ Running the Code

### ✅ Prerequisites

* Both the rover (server) and the base station (client) must be on the **same local network/subnet**.
* You must know the **server's IP address** to configure the client.
* Chrome browser is required for setting "insecure origins treated as secure" if not using HTTPS.

---

### 💻 On the Rover (Server)

1. Install Python dependencies:

   ```bash
   pip install aiohttp aiortc
   ```
2. Run the WebRTC server:

   ```bash
   python3 webrtc.py
   ```

---

### 💼 On the Base Station (Client)

1. Navigate to the React app directory and install dependencies:

   ```bash
   npm install
   ```

2. Update the client code to point to the server's IP address in API requests (e.g., `http://<ROVER_IP>:8081/offer`).

3. Start the client:

   ```bash
   npm run dev
   ```

4. Open Chrome and navigate to:

   ```
   chrome://flags/#unsafely-treat-insecure-origin-as-secure
   ```

   Add both the server address (e.g., `http://<ROVER_IP>:8081`) and the React client address (e.g., `http://localhost:3000`) to the list.

5. Open the app in Chrome. Select a camera stream to initiate a connection.

---

## 🛠️ Notes

* Cameras are expected to be available under Linux device paths like `/dev/video0`, `/dev/video1`, etc.

