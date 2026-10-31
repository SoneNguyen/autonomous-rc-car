"""Keep a Gazebo component demo and its rqt viewers tiled on one monitor."""
from __future__ import annotations

import ctypes
import ctypes.util
from dataclasses import dataclass
import time


@dataclass(frozen=True)
class Window:
    window_id: int
    title: str
    x: int
    y: int


@dataclass(frozen=True)
class Monitor:
    x: int
    y: int
    width: int
    height: int


class _ClientMessageData(ctypes.Union):
    _fields_ = [
        ("b", ctypes.c_char * 20), ("s", ctypes.c_short * 10),
        ("l", ctypes.c_long * 5),
    ]


class _ClientMessageEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int), ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int), ("display", ctypes.c_void_p),
        ("window", ctypes.c_ulong), ("message_type", ctypes.c_ulong),
        ("format", ctypes.c_int), ("data", _ClientMessageData),
    ]


class _XEvent(ctypes.Union):
    _fields_ = [("xclient", _ClientMessageEvent), ("pad", ctypes.c_long * 24)]


class _X11:
    def __init__(self) -> None:
        x11_path = ctypes.util.find_library("X11")
        randr_path = ctypes.util.find_library("Xrandr")
        if not x11_path or not randr_path:
            raise RuntimeError("X11 and XRandR libraries are required to tile demo windows")
        self.x11 = ctypes.CDLL(x11_path)
        self.randr = ctypes.CDLL(randr_path)
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError("could not connect to the X display")

        self.x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self.x11.XDefaultRootWindow.restype = ctypes.c_ulong
        self.root = self.x11.XDefaultRootWindow(self.display)
        self.x11.XFetchName.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_void_p),
        ]
        self.x11.XQueryTree.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)), ctypes.POINTER(ctypes.c_uint),
        ]
        self.x11.XTranslateCoordinates.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong,
            ctypes.c_int, ctypes.c_int,
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_ulong),
        ]
        self.x11.XMoveResizeWindow.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
            ctypes.c_uint, ctypes.c_uint,
        ]
        self.x11.XFlush.argtypes = [ctypes.c_void_p]
        self.x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.x11.XFree.argtypes = [ctypes.c_void_p]
        self.x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        self.x11.XInternAtom.restype = ctypes.c_ulong
        self.x11.XSendEvent.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
            ctypes.c_long, ctypes.POINTER(_XEvent),
        ]
        self.x11.XSendEvent.restype = ctypes.c_int
        self.randr.XRRGetMonitors.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.POINTER(ctypes.c_int),
        ]
        self.randr.XRRGetMonitors.restype = ctypes.POINTER(self._MonitorInfo)
        self.randr.XRRFreeMonitors.argtypes = [ctypes.POINTER(self._MonitorInfo)]

    class _MonitorInfo(ctypes.Structure):
        _fields_ = [
            ("name", ctypes.c_ulong), ("primary", ctypes.c_int),
            ("automatic", ctypes.c_int), ("noutput", ctypes.c_int),
            ("x", ctypes.c_int), ("y", ctypes.c_int),
            ("width", ctypes.c_int), ("height", ctypes.c_int),
            ("mwidth", ctypes.c_int), ("mheight", ctypes.c_int),
            ("outputs", ctypes.POINTER(ctypes.c_ulong)),
        ]

    def _title(self, window_id: int) -> str:
        value = ctypes.c_void_p()
        if not self.x11.XFetchName(self.display, window_id, ctypes.byref(value)) or not value:
            return ""
        try:
            return ctypes.string_at(value).decode("utf-8", errors="replace")
        finally:
            self.x11.XFree(value)

    def windows(self) -> list[Window]:
        result: list[Window] = []

        def visit(window_id: int, depth: int, visited: set[int]) -> None:
            if depth > 4 or window_id in visited:
                return
            visited.add(window_id)
            title = self._title(window_id)
            if title and ("Gazebo Sim" in title or title.startswith((
                "rqt_image_view__", "rqt_plot__", "rqt_topic__",
            ))):
                x, y, child = ctypes.c_int(), ctypes.c_int(), ctypes.c_ulong()
                if self.x11.XTranslateCoordinates(
                    self.display, window_id, self.root, 0, 0,
                    ctypes.byref(x), ctypes.byref(y), ctypes.byref(child),
                ):
                    result.append(Window(window_id, title, x.value, y.value))

            root_return, parent_return = ctypes.c_ulong(), ctypes.c_ulong()
            children = ctypes.POINTER(ctypes.c_ulong)()
            count = ctypes.c_uint()
            if not self.x11.XQueryTree(
                self.display, window_id, ctypes.byref(root_return),
                ctypes.byref(parent_return), ctypes.byref(children), ctypes.byref(count),
            ):
                return
            try:
                for index in range(min(count.value, 256)):
                    visit(int(children[index]), depth + 1, visited)
            finally:
                if children:
                    self.x11.XFree(children)

        root_return, parent_return = ctypes.c_ulong(), ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        count = ctypes.c_uint()
        if not self.x11.XQueryTree(
            self.display, self.root, ctypes.byref(root_return),
            ctypes.byref(parent_return), ctypes.byref(children), ctypes.byref(count),
        ):
            return result
        try:
            visited: set[int] = set()
            for index in range(min(count.value, 256)):
                visit(int(children[index]), 0, visited)
        finally:
            if children:
                self.x11.XFree(children)
        return result

    def monitors(self) -> list[Monitor]:
        count = ctypes.c_int()
        monitors = self.randr.XRRGetMonitors(
            self.display, self.root, 1, ctypes.byref(count),
        )
        if not monitors:
            return []
        try:
            return [
                Monitor(monitors[i].x, monitors[i].y, monitors[i].width, monitors[i].height)
                for i in range(count.value)
            ]
        finally:
            self.randr.XRRFreeMonitors(monitors)

    def move_resize(self, window: Window, x: int, y: int, width: int, height: int) -> None:
        # WSLg's window frame adds roughly 76x97 pixels around each Qt client.
        # Give the client this inset so its visible frame sits inside the tile.
        self.x11.XMoveResizeWindow(
            self.display, window.window_id, x + 32, y + 32,
            max(240, width - 76), max(180, height - 97),
        )
        self.x11.XFlush(self.display)
        self.x11.XSync(self.display, 0)

    def request_close(self, window: Window) -> bool:
        """Ask a Qt window to close using the standard WM_DELETE_WINDOW event."""
        protocols = self.x11.XInternAtom(self.display, b"WM_PROTOCOLS", 0)
        delete_window = self.x11.XInternAtom(self.display, b"WM_DELETE_WINDOW", 0)
        event = _XEvent()
        event.xclient.type = 33  # ClientMessage
        event.xclient.display = self.display
        event.xclient.window = window.window_id
        event.xclient.message_type = protocols
        event.xclient.format = 32
        event.xclient.data.l[0] = delete_window
        event.xclient.data.l[1] = 0
        sent = self.x11.XSendEvent(
            self.display, window.window_id, 0, 0, ctypes.byref(event),
        )
        self.x11.XFlush(self.display)
        return bool(sent)


def tile_component_windows(viewer_packages: list[str], *, retries: int = 40) -> str | None:
    """Tile the Gazebo scene and rqt windows on the Gazebo window's monitor."""
    try:
        x11 = _X11()
    except (OSError, RuntimeError) as exc:
        return f"Could not arrange demo windows: {exc}"

    windows: list[Window] = []
    gazebo: Window | None = None
    viewers: list[Window] = []
    package_prefix = {
        "rqt_image_view": "rqt_image_view__",
        "rqt_plot": "rqt_plot__",
        "rqt_topic": "rqt_topic__",
    }
    for _ in range(retries):
        windows = x11.windows()
        gazebo = next((window for window in windows if "Gazebo Sim" in window.title), None)
        claimed: set[int] = set()
        viewers = []
        for package in viewer_packages:
            prefix = package_prefix.get(package, f"{package}__")
            match = next((
                window for window in windows
                if window.window_id not in claimed and window.title.startswith(prefix)
            ), None)
            if match:
                claimed.add(match.window_id)
                viewers.append(match)
        if gazebo is not None and len(viewers) == len(viewer_packages):
            break
        time.sleep(0.25)

    if gazebo is None:
        return "Could not find the Gazebo window to arrange the demo."
    if len(viewers) != len(viewer_packages):
        return "Could not find every rqt window; leaving the current layout in place."

    monitors = x11.monitors()
    if not monitors:
        return "Could not detect display monitors; leaving the current layout in place."
    monitor = next((
        item for item in monitors
        if item.x <= gazebo.x < item.x + item.width
        and item.y <= gazebo.y < item.y + item.height
    ), monitors[0])

    margin, gap = 12, 8
    usable_width = monitor.width - 2 * margin - gap
    usable_height = monitor.height - 2 * margin
    scene_width = int(usable_width * 0.65)
    panel_x = monitor.x + margin + scene_width + gap
    panel_width = usable_width - scene_width
    x11.move_resize(
        gazebo, monitor.x + margin, monitor.y + margin,
        scene_width, usable_height,
    )
    tile_height = (usable_height - gap * (len(viewers) - 1)) // len(viewers)
    for index, viewer in enumerate(viewers):
        panel_y = monitor.y + margin + index * (tile_height + gap)
        x11.move_resize(viewer, panel_x, panel_y, panel_width, tile_height)

    return (
        f"Aligned Gazebo and {len(viewers)} viewer window(s) on the same display "
        f"({monitor.width}x{monitor.height})."
    )


def request_close_component_windows(viewer_packages: list[str] | None = None) -> int:
    """Request clean closure of this demo's Gazebo and selected rqt windows."""
    try:
        x11 = _X11()
    except (OSError, RuntimeError):
        return 0
    prefixes = {
        "rqt_image_view": "rqt_image_view__",
        "rqt_plot": "rqt_plot__",
        "rqt_topic": "rqt_topic__",
    }
    wanted = {prefixes.get(package, f"{package}__") for package in viewer_packages or []}
    requested = 0
    for window in x11.windows():
        is_gazebo = "Gazebo Sim" in window.title
        is_viewer = any(window.title.startswith(prefix) for prefix in wanted)
        if (is_gazebo or (viewer_packages is not None and is_viewer)) and x11.request_close(window):
            requested += 1
    return requested
