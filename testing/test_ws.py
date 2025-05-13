import websocket
import json

def on_message(ws, message):
    print("Received:", message)
    try:
        json_data = json.loads(message)
        print("Parsed JSON:", json_data)
    except json.JSONDecodeError:
        print("Not valid JSON")

def on_error(ws, error):
    print("Error:", error)

def on_close(ws, close_status_code, close_msg):
    print("Closed")

def on_open(ws):
    print("Connected")

ws = websocket.WebSocketApp("ws://localhost:3000",
                            on_open=on_open,
                            on_message=on_message,
                            on_error=on_error,
                            on_close=on_close)
ws.run_forever()