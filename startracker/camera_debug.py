# from picamera2 import Picamera2, Preview
# import time
# import numpy as np

# picam2 = Picamera2()

# print(picam2.camera_properties)
# print(picam2.sensor_modes)

# camera_config = picam2.create_preview_configuration()
# picam2.configure(camera_config)

# picam2.set_controls({"ExposureTime": 760565, "AnalogueGain": 16.0})

# picam2.start()
# time.sleep(2)

# arrays = []
# for i in range(16):
#     time.sleep(1)
#     print(i)
#     arrays.append(picam2.capture_array("main"))
# arrays = np.sum(arrays, axis=0).astype(np.uint8)

# np.save("test.npy", arrays)

if False:
    # IMX219
    import numpy as np
    import time

    from picamera2 import Picamera2
    from picamera2.sensor_format import SensorFormat

    exposure_time = 200000 # 760565
    num_frames = 8

    # Configure an unpacked raw format as these are easier to add.
    picam2 = Picamera2()
    raw_format = SensorFormat(picam2.sensor_format)
    raw_format.packing = None
    config = picam2.create_still_configuration(raw={"format": raw_format.format}, buffer_count=2)
    picam2.configure(config)
    picam2.set_controls({"ExposureTime": exposure_time, "AnalogueGain": 8.0})
    picam2.start()

    images = []
    for _ in range(num_frames):
        time.sleep(exposure_time / 4e6)
        images.append(picam2.capture_array("raw").view(np.uint16))
    metadata = picam2.capture_metadata()

    accumulated = np.sum(images, axis=0, dtype=np.int64)

    # # Fix the black level, and convert back to uint8 form for saving as a DNG.
    black_level = metadata["SensorBlackLevels"][0] / 2**(16 - raw_format.bit_depth)
    accumulated -= (num_frames - 1) * int(black_level)
    accumulated = np.clip(accumulated, 0, (2 ** 16) - 1).astype(np.uint16)
    # accumulated = accumulated.view(np.uint8)
    # metadata["ExposureTime"] = exposure_time
    # picam2.helpers.save_dng(accumulated, metadata, config["raw"], "accumulated.dng")

    np.save("test.npy", accumulated)

if True:
    # High res cam raw (3040, 6112) uint8

    print("start")

    import numpy as np
    import time
    import cv2

    from picamera2 import Picamera2
    from picamera2.sensor_format import SensorFormat

    # max exposure time:667244877
    exposure_time =   10000000 # 10s (but 50s then 20s, 20s ...)
    exposure_time =   5000000 # 50s (but 25s then 10s, 10s ...)
    num_frames = 3

    # Configure an unpacked raw format as these are easier to add.
    print("init pi cam")
    picam2 = Picamera2()
    raw_format = SensorFormat(picam2.sensor_format)
    raw_format.packing = None
    print(raw_format)
    print(raw_format.__dict__)
    config = picam2.create_still_configuration(buffer_count=1, queue=False)
    picam2.configure(config)
    print("Set exposure")
    picam2.set_controls({"ExposureTime": exposure_time, "AnalogueGain": 1.0})
    picam2.start()

    print("start")

    images = []
    accumulated = None
    for i in range(num_frames):
        t0 = time.time()
        image = picam2.capture_array("raw")
        print("time:", time.time() - t0)
        if accumulated is None:
            accumulated = image.astype(np.int16)
        else:
            accumulated += image
        print(i)

    np.save("test.npy", image)
    # cv2.imwrite("test.jpg", image)

    # metadata = picam2.capture_metadata()
    # # # Fix the black level, and convert back to uint8 form for saving as a DNG.
    # black_level = metadata["SensorBlackLevels"][0] / 2**(16 - raw_format.bit_depth)
    # accumulated -= (num_frames - 1) * int(black_level)
    # accumulated = np.clip(accumulated, 0, (2 ** 16) - 1).astype(np.uint16)
    # # accumulated = accumulated.view(np.uint8)
    # # metadata["ExposureTime"] = exposure_time
    # # picam2.helpers.save_dng(accumulated, metadata, config["raw"], "accumulated.dng")

    # np.save("test2.npy", image)

    print("done")

if False:
    # High res cam color (3040, 4056, 3) uint8

    print("start")

    import numpy as np
    import time
    import cv2

    from picamera2 import Picamera2
    from picamera2.sensor_format import SensorFormat

    exposure_time = 10000000 # max: 667244877

    picam2 = Picamera2()
    config = picam2.create_still_configuration()
    picam2.configure(config)
    picam2.set_controls({"ExposureTime": exposure_time, "AnalogueGain": 1.0})

    picam2.start()
    t0 = time.time()
    image = picam2.capture_array()
    print(time.time() - t0)
    picam2.stop()
    np.save("test.npy", image)

    print("done")
