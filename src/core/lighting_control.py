from vps_lightcontrol import VPS24X04ES_TCP
import logging
from logging_setup import log_config
import time

def initialize_lights():
    psu = VPS24X04ES_TCP("100.100.100.7", 20108)

def set_lights(psu):
    try:
        psu.set_output_master(True) # enable outputs
        psu.set_intensity(1,255)
        psu.set_intensity(2, 50) # vibes

    finally:
        psu.close()
    pass

if __name__ == "__main__":
  set_lights()