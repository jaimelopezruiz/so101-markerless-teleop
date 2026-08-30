from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot_teleoperator_markerless.config import MarkerlessTeleopConfig
from bridge.bridge import RobotArm, M, Blist, limits, THETALIST_PARK, THETALIST_NEUTRAL
from perception.perception_thread import PerceptionThread
import time
import numpy as np

class MarkerlessTeleop(Teleoperator):
    config_class = MarkerlessTeleopConfig
    name = "markerless"

    def __init__(self, config):
        super().__init__(config)
        self.config = config
        self.arm = RobotArm(M, Blist, limits, theta_park=THETALIST_PARK, theta_neutral=THETALIST_NEUTRAL, 
                            min_cutoff=config.min_cutoff, beta=config.beta)
        self.perception = PerceptionThread(camera_index=config.camera_index)
        self._last_frame_id = -1   # last frame get_action() acted on (staleness guard)
        self._armed = False        # ready-pose gate: hold PARK until the operator matches it
        self._ready_count = 0      # consecutive in-tolerance frames so far
        self._last_xz = None       # most recent mapped target, for debug/overlay only
        self._last_gate_print = 0.0   # throttle for the gate's terminal feedback

        # --- control-rate telemetry ---------------------------------------
        # The gap between get_action() calls is the true control period, and it
        # is not the camera period: the record loop paces itself at its own fps
        # while perception runs free. Measuring it here is what the velocity
        # controller will integrate against, and logging the distribution is how
        # dt_max_factor gets checked against real jitter instead of assumed.
        self._dt = None            # seconds since the previous tick; None on tick 0
        self._last_tick = None
        self._dt_max = config.dt_max_factor / config.nominal_fps
        self._tel_t0 = None        # telemetry window start
        self._tel_ticks = 0        # get_action calls this window
        self._tel_frames = 0       # NEW perception frames consumed this window
        self._tel_dts = []         # measured dt samples this window
        self._tel_over = 0         # ticks whose dt exceeded _dt_max

    @property 
    def feedback_features(self) -> dict[str, type]:      # Nothing while we don't have channels back to operator (haptics, force...)
        return {}
    
    @property
    def is_connected(self) -> bool:
        return self.perception.is_running

    @property
    def is_calibrated(self) -> bool:
        return (self.arm.arm_length is not None)

    def configure(self) -> None:
        # Pose seed itself happens in RobotArm.__init__; this re-arms the gate so
        # a reconnect always starts parked and re-earns teleop.
        self._last_frame_id = -1
        self._armed = False
        self._ready_count = 0
        self._dt = None
        self._last_tick = None
        self._tel_t0 = None
        self._tel_ticks = self._tel_frames = self._tel_over = 0
        self._tel_dts = []

    def get_action(self) -> dict[str, float]:
        """One tick. Maps + solves only on a NEW, trustworthy frame; every other
        path falls through to hold-last (theta_prev is already the last good
        pose, since solve() advances it only on IK success)."""
        now = time.perf_counter()
        # Measured control period. None on the first tick, and long whenever the
        # caller stalled; the velocity controller must treat both as
        # discontinuities rather than integrate across them.
        self._dt = None if self._last_tick is None else now - self._last_tick
        self._last_tick = now

        if self.perception.error is not None:
            raise self.perception.error
        snap = self.perception.latest()
        fresh = (snap is not None
                 and snap.frame_id != self._last_frame_id         # stale repeat -> hold
                 and snap.visibility is not None                  # no person -> hold
                 and snap.visibility >= self.config.vis_thresh)   # low confidence -> hold
        if fresh:
            self._last_frame_id = snap.frame_id
            # Map EVERY valid frame, armed or not: the one-euro filter inside
            # map_target must stay sampled at true camera cadence.
            x, z = self.arm.map_target(snap.shoulder, snap.wrist)
            self._last_xz = (x, z)
            if self._armed:
                self.arm.solve(x, z)     # advances arm.theta_prev on IK success
            else:
                self._gate(x, z)

        self._telemetry(now, fresh)
        return self.arm.to_action(self.arm.theta_prev)

    def _telemetry(self, now: float, fresh: bool) -> None:
        """Throttled control-rate line: how often lerobot calls us, how many of
        those ticks carried a NEW perception frame, and the measured dt spread.

        The gap between those first two numbers is the whole reason the staleness
        guard exists, and `over` counts ticks past dt_max_factor x nominal, so the
        skip threshold can be set against observed jitter rather than guessed.
        """
        if self.config.log_every_s <= 0:
            return
        if self._tel_t0 is None:
            self._tel_t0 = now

        self._tel_ticks += 1
        self._tel_frames += int(fresh)
        if self._dt is not None:
            self._tel_dts.append(self._dt)
            if self._dt > self._dt_max:
                self._tel_over += 1

        window = now - self._tel_t0
        if window < self.config.log_every_s or not self._tel_dts:
            return

        dts = np.array(self._tel_dts) * 1e3      # ms
        print(f"[rate] ctrl {self._tel_ticks / window:5.1f} Hz | "
              f"frames {self._tel_frames / window:5.1f} Hz | "
              f"dt {np.median(dts):5.1f} ms med "
              f"({dts.min():.1f}-{dts.max():.1f}) | "
              f"over {self._dt_max * 1e3:.0f} ms: {self._tel_over}")
        self._tel_t0 = now
        self._tel_ticks = self._tel_frames = self._tel_over = 0
        self._tel_dts = []

    def _gate(self, x: float, z: float) -> None:
        """Ready-pose gate. Arms teleop once the mapped target has sat within
        gate_eps of PARK's end-effector for gate_n_frames CONSECUTIVE frames, so
        teleop begins with target ~= the robot's actual pose (no startup lunge).
        Matched in task space, not on 'is the arm folded': fully folded maps to
        r=0.165 m but PARK sits at r=0.206 m, so 'folded' would still jump ~4 cm.
        """
        dist = float(np.linalg.norm(np.array([x, z]) - self.arm.park_xz))
        if dist <= self.config.gate_eps:
            self._ready_count += 1
            if self._ready_count >= self.config.gate_n_frames:
                self._armed = True
                print("teleop ARMED - now following")
                return
        else:
            self._ready_count = 0    # reset: the count must be CONSECUTIVE

        # Terminal feedback, throttled. Without it the operator is hunting a
        # 2.5 cm target blind under lerobot-record, which owns no window of ours.
        # A bare distance says how far but not which way, so report the two knobs
        # the operator actually controls: arm extension and hand direction.
        now = time.perf_counter()
        if now - self._last_gate_print >= 0.5:
            self._last_gate_print = now
            r_now, a_now = self.arm.polar((x, z))
            r_park, a_park = self.arm.polar(self.arm.park_xz)
            print(f"[gate] {dist * 100:5.1f} cm from ready "
                  f"(need <{self.config.gate_eps * 100:.1f}) | "
                  f"reach {r_now * 100:5.1f}->{r_park * 100:.1f} cm "
                  f"({'extend' if r_now < r_park else 'fold'}) | "
                  f"angle {a_now:+6.1f}->{a_park:+.1f} deg "
                  f"({'raise' if a_now < a_park else 'lower'}) | "
                  f"{self._ready_count}/{self.config.gate_n_frames}")

    def send_feedback(self, feedback: dict[str, float]) -> None:    # Will fail "loudly" if send_feedback is called
        raise NotImplementedError

    @property
    def action_features(self) -> dict[str, type]:
        return {k: float for k in self.arm.to_action(self.arm.theta_prev)}

    def calibrate(self) -> None:
        stop = False
        last_id = -1
        self.arm.start_calibration()
        input(f"Extend RIGHT arm towards the front (facing perpendicularly to camera) and press ENTER....")
        while not stop:
            if self.perception.error is not None:
                raise self.perception.error
            if (snap := self.perception.latest()) is not None:
                if snap.frame_id != last_id:
                    last_id = snap.frame_id
                    if snap.visibility is not None and snap.visibility >= self.config.vis_thresh:
                        if self.arm.calibrate(snap.shoulder, snap.wrist, n_samples=self.config.n_calib_frames):
                                stop = True
            time.sleep(0.01)
    
    def connect(self, calibrate: bool = True) -> None:
        self.perception.start()
        if not self.is_calibrated and calibrate:
            self.calibrate()
        self.configure()

    def disconnect(self) -> None:
        self.perception.stop()


# --- demo (needs a camera) 
def _demo() -> None:
    config = MarkerlessTeleopConfig(camera_index=0)
    MTeleop = MarkerlessTeleop(config)
    MTeleop.connect()
    print("Connected: ", MTeleop.is_connected)
    print("Calibrated: ", MTeleop.is_calibrated)
    print("get_action output: ", MTeleop.get_action())
    MTeleop.disconnect()

if __name__ == "__main__": _demo()