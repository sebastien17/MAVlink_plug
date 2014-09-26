import BaseHTTPServer
import SimpleHTTPServer

PORT = 43017

def start_server():
    """Start the server."""
    server_address = ("", PORT)
    server = BaseHTTPServer.HTTPServer(server_address, SimpleHTTPServer.SimpleHTTPRequestHandler)
    server.serve_forever()

if __name__ == "__main__":
    start_server()