#!/usr/bin/python3
# -*- coding: utf-8 -*

import websocket
from datetime import datetime

# Websocket Verbindung
try:
    ws = websocket.create_connection("ws://192.168.1.118/ws", timeout=10)
except:
    ws = websocket.create_connection("ws://192.168.43.55/ws", timeout=10)
csv = open(datetime.now().strftime("log_%Y-%m-%d_%H-%M.csv"), "a")

try:
    while True:
        # per Websocket empfangen
        message = ws.recv()
        if message[0] != "l" or "sensors:" not in message:
            continue
        print(message[:-1])
        # als CSV speichern
        message = message.replace(": ", ";").replace(",", ";")
        csv.write(message)

except (websocket.WebSocketProtocolException) as e:
    print("ws err")
    pass

except KeyboardInterrupt:
    csv.close()
finally:
    csv.close()