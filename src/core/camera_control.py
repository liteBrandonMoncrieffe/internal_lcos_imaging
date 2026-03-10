from pypylon import pylon
import numpy as np
import cv2
import os
from datetime import datetime

def snapshot(save_dir="images"):
    os.makedirs(save_dir, exist_ok=True)

    # Create and open camera
    camera = pylon.InstantCamera(
        pylon.TlFactory.GetInstance().CreateFirstDevice()
    )
    camera.Open()

    # ----- Optional GigE transport tuning -----
    if camera.GetDeviceInfo().GetDeviceClass() == "BaslerGigE":
        camera.GevSCPSPacketSize.SetValue(1500)
        camera.GevSCPD.SetValue(5000)

    # ----- Imaging settings -----
    camera.PixelFormat.SetValue("Mono8")   # change if color
    camera.ExposureAuto.SetValue("Off")
    camera.GainAuto.SetValue("Off")
    camera.ExposureTime.SetValue(10000.0)  # 10 ms
    camera.Gain.SetValue(0.0)

    # ----- Grab one image -----
    camera.StartGrabbingMax(1)
    grab = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if not grab.GrabSucceeded():
        raise RuntimeError(f"Grab failed: {grab.ErrorCode} {grab.ErrorDescription}")

    image = grab.Array.copy()
    grab.Release()
    camera.Close()

    # ----- Save -----
    stamp = len(os.listdir(save_dir)) + 1
    filename = os.path.join(save_dir, f"image_{stamp}.png")

    cv2.imwrite(filename, image)
    print(f"Saved image to {filename}")

if __name__ == "__main__":
    snapshot()