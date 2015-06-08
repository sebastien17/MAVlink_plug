if(__name__ == '__main__'):
    
    
    import mavlinkplug.Module
    import mavlinkplug.Plug
    from time import sleep
    import logging
    
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    handler = logging.StreamHandler()
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter("%(levelname)s - %(message)s")
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    
    test = mavlinkplug.Module.MAVlinkPlugConnection(('tcp://127.0.0.1:45689','tcp://127.0.0.1:45688',123),'COM3',dialect='pixhawk')
    test.start()
    mavlinkplug.Plug.Plug.server_forever()