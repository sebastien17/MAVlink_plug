from  mavlinkplug.modules.JSBSimControl import JSBSimControl
import time

myjsb = JSBSimControl()
myjsb.launch()
myjsb.resume()
time.sleep(600)
myjsb.terminate()