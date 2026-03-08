#Without CAUTION

#!/usr/bin/env python3
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import time
import hailo
import numpy as np
import serial
from collections import OrderedDict

# ==========================================
# 1. RESOURCE CONFIGURATION
# ==========================================
HEF_PATH = "/usr/local/hailo/resources/models/hailo8l/yolov8s.hef"
POST_PROCESS_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"

# ==========================================
# 2. HARDWARE PORT MAPPING
# ==========================================
PORT_MEGA_A = '/dev/arduino/mega_left'
PORT_NANO_A = '/dev/arduino/nano_left'
PORT_MEGA_B = '/dev/arduino/uno_right'
PORT_NANO_B = '/dev/arduino/nano_right'

# ==========================================
# 3. INDEPENDENT CAMERA TUNING
# ==========================================
CAM_CONFIG = {
    0: { 'entry_y': 220, 'exit_y': 300, 'real_dist': 8.0, 'dist_to_curve': 10.0 },
    1: { 'entry_y': 260, 'exit_y': 330, 'real_dist': 5.0, 'dist_to_curve': 10.0 }
}
COOLDOWN_TIME = 2.5 

# ==========================================
# 4. LOGIC CLASSES
# ==========================================
class CentroidTracker:
    def __init__(self, maxDisappeared=10):
        self.nextObjectID = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()
        self.maxDisappeared = maxDisappeared

    def register(self, centroid, bbox):
        self.objects[self.nextObjectID] = (centroid, bbox)
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        if objectID in self.objects:
            del self.objects[objectID]
            del self.disappeared[objectID]

    def update(self, new_detections):
        if len(new_detections) == 0:
            for objectID in list(self.disappeared.keys()):
                self.disappeared[objectID] += 1
                if self.disappeared[objectID] > self.maxDisappeared: self.deregister(objectID)
            return self.objects
        if len(self.objects) == 0:
            for (bbox, centroid) in new_detections: self.register(centroid, bbox)
            return self.objects
        
        objectIDs = list(self.objects.keys())
        objectCentroids = [c for (c, b) in self.objects.values()]
        newCentroids = [c for (b, c) in new_detections]
        D = np.array([np.linalg.norm(np.array(oc) - np.array(nc)) for oc in objectCentroids for nc in newCentroids])
        D = D.reshape((len(objectCentroids), len(newCentroids)))
        rows, cols = D.min(axis=1).argsort(), D.argmin(axis=1)[D.min(axis=1).argsort()]
        usedRows, usedCols = set(), set()
        for (row, col) in zip(rows, cols):
            if row in usedRows or col in usedCols: continue
            objectID = objectIDs[row]
            self.objects[objectID] = (new_detections[col][1], new_detections[col][0])
            self.disappeared[objectID] = 0
            usedRows.add(row); usedCols.add(col)
        for row in set(range(0, D.shape[0])).difference(usedRows):
            objectID = objectIDs[row]
            self.disappeared[objectID] += 1
            if self.disappeared[objectID] > self.maxDisappeared: self.deregister(objectID)
        for col in set(range(0, D.shape[1])).difference(usedCols):
            self.register(new_detections[col][1], new_detections[col][0])
        return self.objects
    
class BlindCurveSystem:
    def __init__(self):
        self.trackers = {0: CentroidTracker(), 1: CentroidTracker()}
        self.timers = {0: {}, 1: {}}
        self.previous_positions = {0: {}, 1: {}}
        self.last_state_a = 'G'
        self.last_state_b = 'G'
        self.m0_confidence = 0
        self.m1_confidence = 0
        self.last_m0_detection_time = 0
        self.last_m1_detection_time = 0

        print("--- Initializing Hardware ---")
        self.mega_a = self.connect_serial(PORT_MEGA_A, "Mega Left")
        self.nano_a = self.connect_serial(PORT_NANO_A, "Nano Left")
        self.mega_b = self.connect_serial(PORT_MEGA_B, "Mega Right")
        self.nano_b = self.connect_serial(PORT_NANO_B, "Nano Right")

    def connect_serial(self, port, name):
        try:
            if os.path.exists(port):
                ser = serial.Serial(port, 9600, timeout=0.1)
                time.sleep(0.5)
                print(f"[SUCCESS] Connected to {name} on {port}")
                return ser
            print(f"[WARNING] Port {port} ({name}) not found.")
            return None
        except Exception as e:
            print(f"[ERROR] Could not connect to {name}: {e}")
            return None
        
    def update_lights(self, m0_moving, m1_moving):
        current_time = time.time()

        # Side A (Cam 1)
        if m1_moving > 0: self.m1_confidence += 1
        else: self.m1_confidence = 0
        if self.m1_confidence >= 3: self.last_m1_detection_time = current_time
        state_a = 'R' if (current_time - self.last_m1_detection_time < COOLDOWN_TIME) else 'G'
        
        if state_a != self.last_state_a:
            print(f"[LIGHT] Side A (Left) changed to: {state_a}")
            if self.nano_a: self.nano_a.write(state_a.encode())
            
            # --- MODIFIED DISPLAY AND LIGHT LOGIC ---
            if self.mega_a:
                if state_a == 'R':
                    self.mega_a.write(b"SLOW DOWN\n")
                else:
                    self.mega_a.write(b"GO AHEAD\n")
            self.last_state_a = state_a

        # Side B (Cam 0)
        if m0_moving > 0: self.m0_confidence += 1
        else: self.m0_confidence = 0
        if self.m0_confidence >= 1: self.last_m0_detection_time = current_time
        state_b = 'R' if (current_time - self.last_m0_detection_time < COOLDOWN_TIME) else 'G'
        
        if state_b != self.last_state_b:
            print(f"[LIGHT] Side B (Right) changed to: {state_b}")
            if self.nano_b: self.nano_b.write(state_b.encode())
            
            # --- MODIFIED: ADDED CAUTION LOGIC ---
            if self.mega_b:
                if state_b == 'R':
                    self.mega_b.write(b"SLOW DOWN\n")
                else:
                    self.mega_b.write(b"GO AHEAD\n")
            self.last_state_b = state_b

    def process_frame_logic(self, cam_id, detections):
        config = CAM_CONFIG[cam_id]
        tracked_objs = self.trackers[cam_id].update(detections)
        current_time = time.time()
        moving_count = 0
        target_mega = self.mega_b if cam_id == 0 else self.mega_a

        for (tid, (centroid, bbox)) in tracked_objs.items():
            cx, py = centroid
            prev = self.previous_positions[cam_id].get(tid)
            is_incoming = False
            
            if prev:
                dy = py - prev[1]
                if dy > (0.8 if cam_id == 0 else 1.2): is_incoming = True
            self.previous_positions[cam_id][tid] = (cx, py)

            if is_incoming:
                moving_count += 1
                if py > config['entry_y'] and py < config['exit_y']:
                    if tid not in self.timers[cam_id]:
                        self.timers[cam_id][tid] = current_time
                        print(f"[ZONE] Cam {cam_id}: Vehicle {tid} entering measurement zone.")
                        if target_mega: target_mega.write(b"SLOW DOWN\n")
                
                elif py >= config['exit_y']:
                    if tid in self.timers[cam_id]:
                        duration = current_time - self.timers[cam_id][tid]
                        if 0.4 < duration < 10.0:
                            speed = (config['real_dist'] / duration) * 3.6
                            eta = config['dist_to_curve'] / (config['real_dist'] / duration)
                            
                            print(f"[DATA] Cam {cam_id} | Speed: {speed:.1f} km/h | ETA: {eta:.1f}s")
                            
                            if target_mega:
                                target_mega.write(f"S:{speed:.0f}\n".encode())
                                time.sleep(0.01)
                                target_mega.write(f"E:{eta:.1f}\n".encode())
                        del self.timers[cam_id][tid]
        return moving_count
    
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None: return Gst.PadProbeReturn.OK
    dets_0, dets_1 = [], []
    caps = pad.get_current_caps()
    width = caps.get_structure(0).get_int('width').value
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    for obj in detections:
        if obj.get_label() in ["car", "motorcycle", "bus", "truck", "bicycle"]:
            bbox = obj.get_bbox()
            if bbox.ymin() < 0.50 and obj.get_confidence() > 0.28:
                dets_0.append(([bbox.xmin()*width, bbox.ymin()*960, bbox.xmax()*width, bbox.ymax()*960], (int((bbox.xmin()+bbox.xmax())*width/2), int(bbox.ymax()*960))))
            elif bbox.ymin() >= 0.50 and obj.get_confidence() > 0.50:
                dets_1.append(([bbox.xmin()*width, (bbox.ymin()-0.5)*960, bbox.xmax()*width, (bbox.ymax()-0.5)*960], (int((bbox.xmin()+bbox.xmax())*width/2), int((bbox.ymax()-0.5)*960))))

    m0 = user_data.process_frame_logic(0, dets_0)
    m1 = user_data.process_frame_logic(1, dets_1)
    user_data.update_lights(m0, m1)
    return Gst.PadProbeReturn.OK

def main():
    Gst.init(None)
    system = BlindCurveSystem()
    pipeline_str = (
        f"libcamerasrc camera-name=/base/axi/pcie@1000120000/rp1/i2c@88000/imx477@1a ! video/x-raw, width=640, height=480, format=NV12 ! queue ! mix.sink_0 "
        f"libcamerasrc camera-name=/base/axi/pcie@1000120000/rp1/i2c@80000/imx477@1a ! video/x-raw, width=640, height=480, format=NV12 ! queue ! mix.sink_1 "
        f"compositor name=mix sink_0::xpos=0 sink_0::ypos=0 sink_1::xpos=0 sink_1::ypos=480 ! "
        f"video/x-raw, width=640, height=960 ! videoscale ! video/x-raw, width=640, height=640 ! "
        f"hailonet hef-path={HEF_PATH} ! hailofilter so-path={POST_PROCESS_SO} function-name=filter_letterbox qos=false ! "
        f"fakesink name=logic_sink"
    )
    pipeline = Gst.parse_launch(pipeline_str)
    pipeline.get_by_name("logic_sink").get_static_pad("sink").add_probe(Gst.PadProbeType.BUFFER, app_callback, system)
    pipeline.set_state(Gst.State.PLAYING)
    try: GLib.MainLoop().run()
    except KeyboardInterrupt: pass
    finally: pipeline.set_state(Gst.State.NULL)

if __name__ == "__main__":
    main()
