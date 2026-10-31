"""RTS-style keyboard controls for the Gazebo scene camera."""
from __future__ import annotations

import ctypes
import ctypes.util
import math
import threading
import time


class _GenericEventCookie(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int), ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int), ("display", ctypes.c_void_p),
        ("extension", ctypes.c_int), ("evtype", ctypes.c_int),
        ("cookie", ctypes.c_uint), ("data", ctypes.c_void_p),
    ]


class _XEvent(ctypes.Union):
    _fields_ = [("type", ctypes.c_int), ("xcookie", _GenericEventCookie),
                ("pad", ctypes.c_long * 24)]


class _XIEventMask(ctypes.Structure):
    _fields_ = [
        ("deviceid", ctypes.c_int), ("mask_len", ctypes.c_int),
        ("mask", ctypes.POINTER(ctypes.c_ubyte)),
    ]


class _XIRawEventPrefix(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int), ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int), ("display", ctypes.c_void_p),
        ("extension", ctypes.c_int), ("evtype", ctypes.c_int),
        ("time", ctypes.c_ulong), ("deviceid", ctypes.c_int),
        ("sourceid", ctypes.c_int), ("detail", ctypes.c_int),
    ]


class GazeboCameraHotkeys:
    """Pan, orbit, and zoom the Gazebo camera while its window has focus."""

    def __init__(self, initial_pose_rpy: tuple[float, ...]) -> None:
        if len(initial_pose_rpy) != 6:
            raise ValueError("initial camera pose must be x, y, z, roll, pitch, yaw")

        from gz import transport
        from gz.msgs import boolean_pb2, cameratrack_pb2, entity_pb2
        from gz.msgs import gui_camera_pb2, pose_pb2, pose_v_pb2

        self._transport = transport
        self._boolean_type = boolean_pb2.Boolean
        self._track_type = cameratrack_pb2.CameraTrack
        self._entity_type = entity_pb2.Entity
        self._gui_camera_type = gui_camera_pb2.GUICamera
        self._pose_type = pose_pb2.Pose
        self._pose_vector_type = pose_v_pb2.Pose_V
        self._node = transport.Node()
        self._track_publisher = self._node.advertise("/gui/track", self._track_type)
        self._pose = self._pose_type()
        self._set_pose_from_rpy(self._pose, initial_pose_rpy)
        self._pose_lock = threading.Lock()
        self._truck_pose_lock = threading.Lock()
        self._follow_lock = threading.Lock()
        self._pose_received = threading.Event()
        self._truck_pose_received = threading.Event()
        self._truck_pose = self._pose_type()
        self._node.subscribe(self._pose_type, "/gui/camera/pose", self._on_pose)
        self._node.subscribe(
            self._pose_vector_type,
            "/world/rc_module_matrix/dynamic_pose/info",
            self._on_world_poses,
        )

        self._display = self._open_display()
        self._gazebo_window = self._find_gazebo_window()
        if not self._gazebo_window:
            raise RuntimeError("could not find the Gazebo window for keyboard controls")

        self._keycodes = self._load_keycodes()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="gazebo-camera-hotkeys", daemon=True)
        self._wheel_thread = threading.Thread(
            target=self._listen_for_wheel, name="gazebo-camera-wheel", daemon=True,
        )
        self._following_truck = False
        self._follow_distance = 3.0
        self._follow_offset = (-3.0, 0.0, 2.0)
        self._initial_pose = self._pose_type()
        self._initial_pose.CopyFrom(self._pose)
        self._last_error_reported = False

    @staticmethod
    def _set_pose_from_rpy(pose, values: tuple[float, ...]) -> None:
        x, y, z, roll, pitch, yaw = values
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.orientation.z = cr * cp * sy - sr * sp * cy
        pose.orientation.w = cr * cp * cy + sr * sp * sy

    def _open_display(self):
        path = ctypes.util.find_library("X11")
        if not path:
            raise RuntimeError("X11 is required for Gazebo keyboard controls")
        x11 = ctypes.CDLL(path)
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XOpenDisplay.restype = ctypes.c_void_p
        display = x11.XOpenDisplay(None)
        if not display:
            raise RuntimeError("could not connect to the X display for keyboard controls")
        self._x11 = x11
        self._x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self._x11.XDefaultRootWindow.restype = ctypes.c_ulong
        self._root = self._x11.XDefaultRootWindow(display)
        self._x11.XFetchName.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_void_p),
        ]
        self._x11.XQueryTree.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)), ctypes.POINTER(ctypes.c_uint),
        ]
        self._x11.XFree.argtypes = [ctypes.c_void_p]
        self._x11.XGetInputFocus.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_int),
        ]
        self._x11.XQueryPointer.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_uint),
        ]
        self._x11.XGetGeometry.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint),
        ]
        self._x11.XTranslateCoordinates.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong,
            ctypes.c_int, ctypes.c_int,
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_ulong),
        ]
        self._x11.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        self._x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self._x11.XStringToKeysym.restype = ctypes.c_ulong
        self._x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self._x11.XKeysymToKeycode.restype = ctypes.c_uint
        self._x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        return display

    def _window_title(self, window_id: int) -> str:
        value = ctypes.c_void_p()
        if not self._x11.XFetchName(self._display, window_id, ctypes.byref(value)) or not value:
            return ""
        try:
            return ctypes.string_at(value).decode("utf-8", errors="replace")
        finally:
            self._x11.XFree(value)

    def _find_gazebo_window(self) -> int:
        root_return, parent_return = ctypes.c_ulong(), ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        count = ctypes.c_uint()
        if not self._x11.XQueryTree(
            self._display, self._root, ctypes.byref(root_return),
            ctypes.byref(parent_return), ctypes.byref(children), ctypes.byref(count),
        ):
            return 0

        def visit(window_id: int, depth: int) -> int:
            if depth > 4:
                return 0
            if "Gazebo Sim" in self._window_title(window_id):
                return window_id
            subroot, parent = ctypes.c_ulong(), ctypes.c_ulong()
            subchildren = ctypes.POINTER(ctypes.c_ulong)()
            subcount = ctypes.c_uint()
            if not self._x11.XQueryTree(
                self._display, window_id, ctypes.byref(subroot), ctypes.byref(parent),
                ctypes.byref(subchildren), ctypes.byref(subcount),
            ):
                return 0
            try:
                for index in range(min(subcount.value, 256)):
                    found = visit(int(subchildren[index]), depth + 1)
                    if found:
                        return found
            finally:
                if subchildren:
                    self._x11.XFree(subchildren)
            return 0

        try:
            for index in range(min(count.value, 256)):
                found = visit(int(children[index]), 0)
                if found:
                    return found
        finally:
            if children:
                self._x11.XFree(children)
        return 0

    def _load_keycodes(self) -> dict[str, int]:
        names = {
            "w": "w", "a": "a", "s": "s", "d": "d", "q": "q", "e": "e",
            "toggle_follow": "t", "reset": "0", "zoom_in": "Prior",
            "zoom_out": "Next", "plus": "equal", "minus": "minus",
            "shift": "Shift_L", "shift_right": "Shift_R",
        }
        result = {}
        for name, keysym_name in names.items():
            keysym = self._x11.XStringToKeysym(keysym_name.encode("ascii"))
            keycode = self._x11.XKeysymToKeycode(self._display, keysym) if keysym else 0
            if keycode:
                result[name] = keycode
        return result

    def _focused_in_gazebo(self) -> bool:
        focus, revert_to = ctypes.c_ulong(), ctypes.c_int()
        self._x11.XGetInputFocus(
            self._display, ctypes.byref(focus), ctypes.byref(revert_to),
        )
        window_id = int(focus.value)
        while window_id not in {0, 1, int(self._root)}:
            if window_id == self._gazebo_window:
                return True
            root_return, parent = ctypes.c_ulong(), ctypes.c_ulong()
            children = ctypes.POINTER(ctypes.c_ulong)()
            count = ctypes.c_uint()
            if not self._x11.XQueryTree(
                self._display, window_id, ctypes.byref(root_return), ctypes.byref(parent),
                ctypes.byref(children), ctypes.byref(count),
            ):
                return False
            if children:
                self._x11.XFree(children)
            if parent.value == window_id:
                return False
            window_id = int(parent.value)
        # Some WSLg window managers report no X keyboard focus even while the
        # pointer is inside an active Gazebo surface.
        return self._pointer_inside_gazebo()

    def _pointer_inside_gazebo(self, display=None) -> bool:
        display = display or self._display
        root_return, child_return = ctypes.c_ulong(), ctypes.c_ulong()
        root_x, root_y, win_x, win_y = ctypes.c_int(), ctypes.c_int(), ctypes.c_int(), ctypes.c_int()
        mask = ctypes.c_uint()
        if not self._x11.XQueryPointer(
            display, self._root, ctypes.byref(root_return), ctypes.byref(child_return),
            ctypes.byref(root_x), ctypes.byref(root_y), ctypes.byref(win_x), ctypes.byref(win_y),
            ctypes.byref(mask),
        ):
            return False
        window_root, x, y = ctypes.c_ulong(), ctypes.c_int(), ctypes.c_int()
        width, height, border, depth = ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint()
        if not self._x11.XGetGeometry(
            display, self._gazebo_window, ctypes.byref(window_root), ctypes.byref(x),
            ctypes.byref(y), ctypes.byref(width), ctypes.byref(height), ctypes.byref(border),
            ctypes.byref(depth),
        ):
            return False
        scene_x, scene_y, child = ctypes.c_int(), ctypes.c_int(), ctypes.c_ulong()
        if not self._x11.XTranslateCoordinates(
            display, self._gazebo_window, self._root, 0, 0,
            ctypes.byref(scene_x), ctypes.byref(scene_y), ctypes.byref(child),
        ):
            return False
        return (
            scene_x.value <= root_x.value < scene_x.value + width.value
            and scene_y.value <= root_y.value < scene_y.value + height.value
        )

    def _listen_for_wheel(self) -> None:
        xi_path = ctypes.util.find_library("Xi")
        if not xi_path:
            return
        xi = ctypes.CDLL(xi_path)
        x11 = self._x11
        display = x11.XOpenDisplay(None)
        if not display:
            return
        xi.XIQueryVersion.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ]
        xi.XISelectEvents.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(_XIEventMask), ctypes.c_int,
        ]
        x11.XQueryExtension.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ]
        x11.XNextEvent.argtypes = [ctypes.c_void_p, ctypes.POINTER(_XEvent)]
        x11.XPending.argtypes = [ctypes.c_void_p]
        x11.XFlush.argtypes = [ctypes.c_void_p]
        x11.XGetEventData.argtypes = [ctypes.c_void_p, ctypes.POINTER(_GenericEventCookie)]
        x11.XFreeEventData.argtypes = [ctypes.c_void_p, ctypes.POINTER(_GenericEventCookie)]
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]

        opcode, first_event, first_error = ctypes.c_int(), ctypes.c_int(), ctypes.c_int()
        if not x11.XQueryExtension(
            display, b"XInputExtension", ctypes.byref(opcode),
            ctypes.byref(first_event), ctypes.byref(first_error),
        ):
            x11.XCloseDisplay(display)
            return
        major, minor = ctypes.c_int(2), ctypes.c_int(0)
        if xi.XIQueryVersion(display, ctypes.byref(major), ctypes.byref(minor)) != 0:
            x11.XCloseDisplay(display)
            return
        mask_storage = (ctypes.c_ubyte * 2)()
        mask_storage[15 // 8] |= 1 << (15 % 8)  # XI_RawButtonPress
        event_mask = _XIEventMask(1, len(mask_storage), mask_storage)
        if xi.XISelectEvents(display, self._root, ctypes.byref(event_mask), 1) != 0:
            x11.XCloseDisplay(display)
            return
        x11.XFlush(display)

        while not self._stop.is_set():
            if x11.XPending(display) == 0:
                self._stop.wait(0.01)
                continue
            event = _XEvent()
            x11.XNextEvent(display, ctypes.byref(event))
            cookie = event.xcookie
            if event.type != 35 or cookie.extension != opcode.value:
                continue
            if not x11.XGetEventData(display, ctypes.byref(event.xcookie)):
                continue
            try:
                if cookie.evtype != 15:  # XI_RawButtonPress
                    continue
                raw = ctypes.cast(
                    event.xcookie.data, ctypes.POINTER(_XIRawEventPrefix),
                ).contents
                if raw.detail in {4, 5} and self._pointer_inside_gazebo(display) and self._following_truck:
                    # Gazebo's native wheel handler already moves the camera.
                    # Adopt that new camera-to-truck distance after the event
                    # so tracking retains the zoom without applying a second
                    # wheel step of its own.
                    self._sync_follow_offset_to_camera()
            finally:
                x11.XFreeEventData(display, ctypes.byref(event.xcookie))
        x11.XCloseDisplay(display)

    def _pressed(self) -> set[str]:
        keymap = ctypes.create_string_buffer(32)
        self._x11.XQueryKeymap(self._display, keymap)
        pressed = set()
        for name, keycode in self._keycodes.items():
            if keymap.raw[keycode // 8] & (1 << (keycode % 8)):
                pressed.add(name)
        if "plus" in pressed:
            pressed.add("zoom_in")
        if "minus" in pressed:
            pressed.add("zoom_out")
        return pressed

    def _on_pose(self, pose) -> None:
        with self._pose_lock:
            self._pose.CopyFrom(pose)
            self._pose_received.set()

    def _on_world_poses(self, poses) -> None:
        for pose in poses.pose:
            if pose.name == "gazebo_truck":
                with self._truck_pose_lock:
                    self._truck_pose.CopyFrom(pose)
                    self._truck_pose_received.set()
                return

    def start(self, *, timeout_s: float = 8.0) -> None:
        if not self._pose_received.wait(timeout_s):
            raise RuntimeError("Gazebo did not publish /gui/camera/pose")
        if not self._truck_pose_received.wait(timeout_s):
            raise RuntimeError(
                "Gazebo did not publish the truck pose on "
                "/world/rc_module_matrix/dynamic_pose/info"
            )
        self._thread.start()
        self._wheel_thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._wheel_thread.is_alive():
            self._wheel_thread.join(timeout=2.0)
        self._x11.XCloseDisplay(self._display)
        self._node.unsubscribe("/gui/camera/pose")
        self._node.unsubscribe("/world/rc_module_matrix/dynamic_pose/info")

    def _set_follow(self, enabled: bool) -> None:
        message = self._track_type()
        if enabled:
            # Capture the existing camera-to-truck offset so enabling follow
            # retains the current zoom. CameraTrack offsets are in the
            # target's local frame, so convert from world coordinates first.
            offset, distance = self._camera_offset_from_truck()
            if distance < 0.1:
                print(
                    "Cannot enable truck-follow while the camera is at the truck origin.",
                    flush=True,
                )
                return
            with self._follow_lock:
                self._follow_offset = offset
                self._follow_distance = distance
            message.track_mode = self._track_type.FOLLOW_LOOK_AT
            message.follow_target.name = "gazebo_truck"
            message.follow_target.type = self._entity_type.MODEL
            message.track_target.name = "gazebo_truck"
            message.track_target.type = self._entity_type.MODEL
            message.follow_offset.x = offset[0]
            message.follow_offset.y = offset[1]
            message.follow_offset.z = offset[2]
            message.track_offset.x = 0.0
            message.track_offset.y = 0.0
            message.track_offset.z = 0.0
            message.follow_pgain = 0.01
            message.track_pgain = 0.01
        else:
            message.track_mode = self._track_type.NONE
        self._track_publisher.publish(message)
        self._following_truck = enabled
        print(
            "Camera following the truck (T toggles free camera)." if enabled
            else "Free camera enabled.",
            flush=True,
        )

    def _camera_offset_from_truck(self) -> tuple[tuple[float, float, float], float]:
        with self._pose_lock:
            camera = self._pose_type()
            camera.CopyFrom(self._pose)
        with self._truck_pose_lock:
            truck = self._pose_type()
            truck.CopyFrom(self._truck_pose)
        world_offset = (
            camera.position.x - truck.position.x,
            camera.position.y - truck.position.y,
            camera.position.z - truck.position.z,
        )
        inverse_rotation = self._pose_type().orientation
        inverse_rotation.x = -truck.orientation.x
        inverse_rotation.y = -truck.orientation.y
        inverse_rotation.z = -truck.orientation.z
        inverse_rotation.w = truck.orientation.w
        offset = self._rotate_vector(inverse_rotation, world_offset)
        distance = math.sqrt(sum(component * component for component in offset))
        return offset, distance

    def _sync_follow_offset_to_camera(self) -> None:
        if self._stop.wait(0.08) or not self._following_truck:
            return
        offset, distance = self._camera_offset_from_truck()
        if distance < 0.1:
            return
        with self._follow_lock:
            self._follow_offset = offset
            self._follow_distance = distance
        message = self._track_type()
        message.track_mode = self._track_type.USE_LAST
        message.follow_offset.x = offset[0]
        message.follow_offset.y = offset[1]
        message.follow_offset.z = offset[2]
        self._track_publisher.publish(message)

    def _update_follow_distance(self, delta_m: float) -> None:
        with self._follow_lock:
            self._follow_distance = min(40.0, max(0.5, self._follow_distance + delta_m))
            current_distance = math.sqrt(
                sum(component * component for component in self._follow_offset)
            )
            scale = self._follow_distance / current_distance
            self._follow_offset = tuple(component * scale for component in self._follow_offset)
            offset = self._follow_offset
        message = self._track_type()
        message.track_mode = self._track_type.USE_LAST
        message.follow_offset.x = offset[0]
        message.follow_offset.y = offset[1]
        message.follow_offset.z = offset[2]
        self._track_publisher.publish(message)

    @staticmethod
    def _rotate_vector(q, vector: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z, w = q.x, q.y, q.z, q.w
        vx, vy, vz = vector
        return (
            (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy + 2 * (x * z + y * w) * vz,
            2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - x * w) * vz,
            2 * (x * z - y * w) * vx + 2 * (y * z + x * w) * vy + (1 - 2 * (x * x + y * y)) * vz,
        )

    @staticmethod
    def _yaw_pose(pose, yaw_delta: float) -> None:
        half = yaw_delta / 2.0
        z, w = math.sin(half), math.cos(half)
        q = pose.orientation
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
            -z * q.y + w * q.x,
            z * q.x + w * q.y,
            z * q.w + w * q.z,
            -z * q.z + w * q.w,
        )

    def _request_pose(self, pose, duration_s: float) -> bool:
        request = self._gui_camera_type()
        request.pose.CopyFrom(pose)
        request.duration = duration_s
        try:
            sent, response = self._node.request(
                "/gui/move_to/pose", request, self._gui_camera_type,
                self._boolean_type, timeout=150,
            )
        except Exception as exc:
            if not self._last_error_reported:
                print(f"Camera key controls unavailable: {exc}", flush=True)
                self._last_error_reported = True
            return False
        if not sent or not response.data:
            if not self._last_error_reported:
                print("Camera key controls could not move the Gazebo view.", flush=True)
                self._last_error_reported = True
            return False
        with self._pose_lock:
            self._pose.CopyFrom(pose)
        return True

    def _reset_view(self) -> None:
        self._set_follow(False)
        self._request_pose(self._initial_pose, 0.35)
        print("Camera returned to the scenario overview.", flush=True)

    def _apply_movement(self, keys: set[str], dt: float) -> None:
        movement = keys & {"w", "a", "s", "d", "q", "e", "zoom_in", "zoom_out"}
        if not movement:
            return
        if self._following_truck:
            camera_motion = movement - {"zoom_in", "zoom_out"}
            if not camera_motion:
                zoom_direction = int("zoom_out" in movement) - int("zoom_in" in movement)
                if zoom_direction:
                    self._update_follow_distance(zoom_direction * 2.5 * dt)
                return
            self._set_follow(False)

        with self._pose_lock:
            pose = self._pose_type()
            pose.CopyFrom(self._pose)
        q = pose.orientation
        forward = self._rotate_vector(q, (1.0, 0.0, 0.0))
        right = self._rotate_vector(q, (0.0, 1.0, 0.0))
        forward_length = math.hypot(forward[0], forward[1]) or 1.0
        right_length = math.hypot(right[0], right[1]) or 1.0
        speed = 4.0 * (3.0 if "shift" in keys or "shift_right" in keys else 1.0)
        dx, dy, dz = 0.0, 0.0, 0.0

        if "w" in keys:
            dx += forward[0] / forward_length * speed * dt
            dy += forward[1] / forward_length * speed * dt
        if "s" in keys:
            dx -= forward[0] / forward_length * speed * dt
            dy -= forward[1] / forward_length * speed * dt
        if "d" in keys:
            dx += right[0] / right_length * speed * dt
            dy += right[1] / right_length * speed * dt
        if "a" in keys:
            dx -= right[0] / right_length * speed * dt
            dy -= right[1] / right_length * speed * dt
        if "zoom_in" in keys:
            dz += forward[2] * speed * dt
            dx += forward[0] * speed * dt
            dy += forward[1] * speed * dt
        if "zoom_out" in keys:
            dz -= forward[2] * speed * dt
            dx -= forward[0] * speed * dt
            dy -= forward[1] * speed * dt
        pose.position.x += dx
        pose.position.y += dy
        pose.position.z = max(0.4, pose.position.z + dz)

        if "q" in keys:
            self._yaw_pose(pose, 0.9 * dt)
        if "e" in keys:
            self._yaw_pose(pose, -0.9 * dt)
        self._request_pose(pose, min(dt * 1.5, 0.08))

    def _run(self) -> None:
        previous = time.monotonic()
        previous_keys: set[str] = set()
        while not self._stop.is_set():
            now = time.monotonic()
            dt = min(now - previous, 0.1)
            previous = now
            if self._focused_in_gazebo():
                keys = self._pressed()
            else:
                keys = set()
            newly_pressed = keys - previous_keys
            if "toggle_follow" in newly_pressed:
                self._set_follow(not self._following_truck)
            if "reset" in newly_pressed:
                self._reset_view()
            self._apply_movement(keys, dt)
            previous_keys = keys
            self._stop.wait(1.0 / 30.0)
