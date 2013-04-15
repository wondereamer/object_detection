from SimpleXMLRPCServer import SimpleXMLRPCServer
from m_util import log,Logger
log.add_level(Logger.INFO)

# A simple server with simple arithmetic functions
server = SimpleXMLRPCServer(("localhost", 8000))
print "Listening on port 8000..."
server.register_multicall_functions()
#server.register_function(draw_vectors_2D, 'draw_vectors_2D')
#server.register_function(draw_force_field, 'draw_force_field')
#server.serve_forever()
class RpcServer(object):
    """docstring for RpcServer"""
    def __init__(self, port = 8000, ip = "localhost"):
        _ip = ip
        _port = port
        _server = SimpleXMLRPCServer((ip, port))
    def register_function(func, funcName):
        '''docstring for register_function''' 
        _server.register_function(func, funcName)
    def run(self):
        '''docstring for run''' 
        server.serve_forever()
RpcServer rpcServer


    
__all__ = ["RpcServer", "rpcServer"]

