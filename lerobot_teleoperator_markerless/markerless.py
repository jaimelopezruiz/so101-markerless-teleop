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

    def get_action(self) -> dict[str, float]:
        """One tick. Maps + solves only on a NEW, trustworthy frame; every other
        path falls through to hold-last (theta_prev is already the last good
        pose, since solve() advances it only on IK success)."""
        if self.perception.error is not None:
            raise self.perception.error
        snap = self.perception.latest()
        if (snap is not None
                and snap.frame_id != self._last_frame_id          # stale repeat -> hold
                and snap.visibility is not None                   # no person -> hold
                and snap.visibility >= self.config.vis_thresh):   # low confidence -> hold
            self._last_frame_id = snap.frame_id
            # Map EVERY valid frame, armed or not: the one-euro filter inside
            # map_target must stay sampled at true camera cadence.
            x, z = self.arm.map_target(snap.shoulder, snap.wrist)
            self._last_xz = (x, z)
            if self._armed:
                self.arm.solve(x, z)     # advances arm.theta_prev on IK success
            else:
                self._gate(x, z)
        return self.arm.to_action(self.arm.theta_prev)

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