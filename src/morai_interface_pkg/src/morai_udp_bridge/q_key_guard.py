"""Read only Q while MORAI is focused to allow local mode handover."""
import ctypes as C
import threading
import time


class ClassHint(C.Structure):
    _fields_ = [("name", C.c_void_p), ("kind", C.c_void_p)]


class QKeyGuard:
    def __init__(self, hold_sec=0.5):
        self.hold_sec, self.until = hold_sec, 0.0
        self.autonomous = True
        self.was_down = False
        self.lock = threading.Lock()
        self.x = C.CDLL("libX11.so.6")
        signatures = {
            "XOpenDisplay": ([C.c_char_p], C.c_void_p),
            "XKeysymToKeycode": ([C.c_void_p, C.c_ulong], C.c_uint),
            "XGetInputFocus": ([C.c_void_p, C.POINTER(C.c_ulong), C.POINTER(C.c_int)], C.c_int),
            "XGetClassHint": ([C.c_void_p, C.c_ulong, C.POINTER(ClassHint)], C.c_int),
            "XQueryKeymap": ([C.c_void_p, C.c_void_p], C.c_int),
            "XFree": ([C.c_void_p], C.c_int),
            "XCloseDisplay": ([C.c_void_p], C.c_int),
        }
        for name, (args, result) in signatures.items():
            getattr(self.x, name).argtypes = args
            getattr(self.x, name).restype = result
        self.display = self.x.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError("Q guard requires the simulator X11 display")
        self.key = self.x.XKeysymToKeycode(self.display, ord('q'))
        self.stop = threading.Event()
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def blocked(self):
        with self.lock:
            focus, revert = C.c_ulong(), C.c_int()
            self.x.XGetInputFocus(self.display, C.byref(focus), C.byref(revert))
            hint = ClassHint()
            simulator = False
            if focus.value > 1 and self.x.XGetClassHint(self.display, focus, C.byref(hint)):
                simulator = bool(hint.kind and C.string_at(hint.kind) == b'Simulator.x86_64')
                if hint.name:
                    self.x.XFree(hint.name)
                if hint.kind:
                    self.x.XFree(hint.kind)
            keys = (C.c_ubyte * 32)()
            self.x.XQueryKeymap(self.display, keys)
            down = bool(keys[self.key // 8] & (1 << (self.key % 8)))
            return self._update_key(down, simulator, time.monotonic())

    def _update_key(self, down, simulator, now):
        if simulator and down:
            if not self.was_down:
                self.autonomous = not self.autonomous
            self.until = now + self.hold_sec
        self.was_down = down
        return not self.autonomous or now < self.until

    def _run(self):
        while not self.stop.wait(0.005):
            self.blocked()

    def close(self):
        self.stop.set()
        self.worker.join()
        self.x.XCloseDisplay(self.display)
