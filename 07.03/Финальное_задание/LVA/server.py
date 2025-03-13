import asyncio
import websockets

class WebSocketServer:
    def __init__(self, host='0.0.0.0'):
        ports = [8766, 8765]
        self.host = host
        self.connected_clients = {port: set() for port in ports}
        self.servers = [] 
        asyncio.run(self.run(ports))

    async def echo(self, websocket, port):
        self.connected_clients[port].add(websocket)
        try:
            async for message in websocket:
                print(message)
                await asyncio.gather(
                    *(client.send(message) for client in self.connected_clients[port] if client != websocket)
                )
        except websockets.exceptions.ConnectionClosed:
            print("Клиент отключился")
        finally:
            self.connected_clients[port].remove(websocket)

    async def start_server(self, port):
        server = await websockets.serve(lambda ws, path: self.echo(ws, port), self.host, port)
        self.servers.append(server)
        return server

    async def run(self, ports):
        await asyncio.gather(*[self.start_server(port) for port in ports])
        print("Сервер запущен")
        await asyncio.gather(*[server.wait_closed() for server in self.servers])

if __name__ == "__main__":
    wss = WebSocketServer()