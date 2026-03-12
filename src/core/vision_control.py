import os

def detect_drop_areas(image_folder = 'images', results_folder = 'output_contours'):
    pass

def detect_drop_areas_1(picture_save_folder, run_id, timestamp, results_folder = "C:\\testsw\\robotic_epoxy_squeeze_drop\\drop_detect_results"):
    """
    Runs drop_detect on the 2 dispense point images, cropping to the middle 3000x3000 pixels,
    and returns the detected areas.

    **Inputs:**
    - picture_save_folder (str): Folder where images are saved.
    - run_id (int): The incrementing ID number used in filenames.
    - timestamp (str): The timestamp used in filenames.
    - results_folder (str): Folder to save drop_detect results.

    **Outputs:**
    - areas (List[float]): List of detected drop areas for each dispense point image.
    """
    # Calculate crop coordinates for center 3000x3000 region
    img_w, img_h = 5472, 3648
    crop_w, crop_h = 3000, 3000
    x1 = (img_w - crop_w) // 2
    y1 = (img_h - crop_h) // 2
    x2 = x1 + crop_w
    y2 = y1 + crop_h

    # init contour deposit group
    os.makedirs(results_folder, exist_ok=True)

    areas = []
    for i in range(1, 3):
        if os.path.exists(picture_save_folder):
            image_path = os.path.join(
                picture_save_folder,
                f"dispense_point_{i}_{run_id}-{timestamp}.png"
            )
            if os.path.exists(image_path):
                # Read, crop, and downscale the image
                img = cv.imread(image_path, cv.IMREAD_GRAYSCALE)
                img_cropped = img[y1:y2, x1:x2]
                img_downscaled = cv.resize(img_cropped, (750, 750), interpolation=cv.INTER_AREA)
                # Save to a temporary file
                with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as tmp:
                    tmp_path = tmp.name
                    cv.imwrite(tmp_path, img_downscaled)
                # Run drop_detect on the downscaled image
                area = drop_detect(tmp_path, 0, 0, 750, 750, run_id * 10 + i, results_folder)
                areas.append(area)
                os.remove(tmp_path)
            else:
                areas.append(None)
    return areas