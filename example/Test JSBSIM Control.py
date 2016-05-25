from  mavlinkplug.modules.JSBSimControl import JSBSimControl
import time

myjsb = JSBSimControl()
myjsb.launch()
time.sleep(5)
myjsb.terminate()