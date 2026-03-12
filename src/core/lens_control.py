from optoICC import connect

def initialize_lens(channel_index=0):
    """
    Connects to the Optotune lens and initializes it for use.
    Args:
        channel_index (int): Channel index where the lens is connected (default 0).
    """
    icc4c = connect()
    icc4c.MiscFeatures.GetDeviceType(channel_index)
    icc4c_lens_channel = getattr(icc4c, f'Channel_{channel_index}')
    icc4c_lens_channel.StaticInput.SetAsInput()
    print(f"Initialized lens on channel {channel_index}")
    return icc4c

def set_lens_focal_power(icc4c, channel_index, focal_power_dpt):
	"""
	Sets the focal power of the Optotune lens to the specified diopters.
	Args:
		icc4c: The connected Optotune ICC4C instance.
		channel_index (int): Channel index where the lens is connected (default 0).
		focal_power_dpt (float): Desired focal power in diopters (default 12).
	"""
	icc4c_lens_channel = getattr(icc4c, f'Channel_{channel_index}')
	icc4c_lens_channel.StaticInput.SetAsInput()
	icc4c_lens_channel.StaticInput.SetFocalPower(focal_power_dpt)
	print(f"Set focal power to {focal_power_dpt} dpt on channel {channel_index}")

def close_lens(icc4c):
    """
    Disconnects from the Optotune lens.
    Args:
        icc4c: The connected Optotune ICC4C instance.
    """
    icc4c.disconnect()
    print("Disconnected from lens")
     
if __name__ == "__main__":
    icc4c = initialize_lens(channel_index=0)
    set_lens_focal_power(icc4c, channel_index=0, focal_power_dpt=0)
    close_lens(icc4c)