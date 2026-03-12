from mudi_http import CM4

def connect(ip_address, port):
    dispenser = CM4(ip_address, port)
    return dispenser

if __name__ == "__main__":
    d = connect("192.168.0.10", 1026)
    d.set_channel(4)
    d.set_parameters(60.0, 2, 200)
    d.dispense()
