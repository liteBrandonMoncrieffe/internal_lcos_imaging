from vps_lightcontrol import VPS24X04ES_TCP
import logging
from logging_setup import log_config
import time

psu = VPS24X04ES_TCP("100.100.100.7", 20108)

try:
  psu.set_output_master(True) # enable outputs
  psu.set_intensity(1,255)
  psu.set_intensity(2, 50) # vibes



finally:
  psu.close()
