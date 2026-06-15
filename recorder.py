import subprocess
import time
import signal
import cv2
import numpy as np
import csv
import sys
import threading
from pyorbbecsdk import Pipeline, Config, OBStreamType, OBFormat, OBAlignMode, OBSensorType

def start_synchronized_teaching():
    # 1. Initialize camera
    print("Initializing Orbbec Femto Mega camera...")
    pipeline = Pipeline()
    config = Config()
    
    profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    color_profile = profile_list.get_video_stream_profile(1920, 1080, OBFormat.RGB, 30)
    
    config.enable_stream(color_profile)
    config.set_align_mode(OBAlignMode.HW_MODE)
    pipeline.start(config)

    # Prepare timestamp CSV
    csv_file = open('camera_timestamps.csv', 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['frame_index', 'frame_timestamp'])

    video_writer = None

    cpp_executable = "./build/path_recorder" 
    robot_ip = "192.168.3.100"  
    robot_csv = "robot_states.csv"
    
    print("\nStarting C++ robot base control program...")
    
    # 2. Launch C++ subprocess and pipe I/O
    robot_process = subprocess.Popen(
        [cpp_executable, robot_ip, robot_csv],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )

    # 3. Async event to catch the ready signal
    is_cpp_ready = threading.Event()

    def stdout_listener():
        while robot_process.poll() is None:
            line = robot_process.stdout.readline()
            if not line: break
            
            print(f"[C++] {line.strip()}") 
            
            # Catching either English or Chinese ready string for backwards compatibility
            if "Ready" in line:
                is_cpp_ready.set()
        
    listener_thread = threading.Thread(target=stdout_listener, daemon=True)
    listener_thread.start()

    is_cpp_ready.wait()

    # 4. Sync Trigger
    input("Robot and camera are ready!\n    Press [Enter] to start synchronized recording...")
    
    # Send newline to unblock C++ 1000Hz control loop
    robot_process.stdin.write('\n')
    robot_process.stdin.flush()

    # 5. Input forwarder thread
    def input_forwarder():
        while robot_process.poll() is None:
            try:
                user_cmd = sys.stdin.readline()
                if user_cmd:
                    robot_process.stdin.write(user_cmd)
                    robot_process.stdin.flush()
            except:
                break
    threading.Thread(target=input_forwarder, daemon=True).start()

    # 6. Main loop: Write AVI video and timestamps
    frame_idx = 0
    try:
        while robot_process.poll() is None:
            frames = pipeline.wait_for_frames(100)
            if frames is None: continue
            
            color_frame = frames.get_color_frame()
            if color_frame is None: continue

            current_time = time.time()

            width = color_frame.get_width()
            height = color_frame.get_height()
            color_data = np.asanyarray(color_frame.get_data())
            color_image = color_data.reshape((height, width, 3))
            
            bgr_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            if video_writer is None:
                video_writer = cv2.VideoWriter(
                    "teaching_demo_01.avi", 
                    cv2.VideoWriter_fourcc(*'MJPG'), 
                    30, 
                    (width, height)
                )
            
            video_writer.write(bgr_image)
            csv_writer.writerow([frame_idx, current_time])
            frame_idx += 1

    except KeyboardInterrupt:
        pass

    finally:
        # 7. Graceful termination and wait for C++ offline computation
        if robot_process.poll() is None:
            try:
                robot_process.stdin.write('q\n')
                robot_process.stdin.flush()
                robot_process.wait(timeout=20) 
            except subprocess.TimeoutExpired:
                print("Warning: C++ offline computation timed out. Terminating process.")
                robot_process.kill()

        pipeline.stop()
        if video_writer is not None:
            video_writer.release()
        csv_file.close()
        print("Camera video, timestamps, robot CSV, and CITR NPY safely saved!")

if __name__ == "__main__":
    start_synchronized_teaching()