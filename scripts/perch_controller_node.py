#!/usr/bin/env python3

import math
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion, PoseStamped

ALIGNING    = 0
APPROACHING = 1
PERCHING    = 2
DONE        = 3

_STAGE_NAMES = {ALIGNING: "ALIGNING", APPROACHING: "APPROACHING", PERCHING: "PERCHING", DONE: "DONE"}

# ── Stage enable flags ────────────────────────────────────────────────────────
# Set a stage to False to skip it entirely.
# Skipped stages are bypassed: the state machine jumps to the next enabled stage.
ENABLED_STAGES = {
    ALIGNING:    True,
    APPROACHING: True,
    PERCHING:    True,
}
# ─────────────────────────────────────────────────────────────────────────────


def _next_enabled(from_stage):
    """Return the next enabled stage after from_stage, or DONE if none remain."""
    for s in [ALIGNING, APPROACHING, PERCHING]:
        if s > from_stage and ENABLED_STAGES.get(s, True):
            return s
    return DONE


def _first_enabled():
    for s in [ALIGNING, APPROACHING, PERCHING]:
        if ENABLED_STAGES.get(s, True):
            return s
    return DONE


class PerchController:

    def __init__(self):
        rospy.init_node('perch_controller_node')

        self.gain       = rospy.get_param('~gain',       0.005)  # m per error unit
        self.z_step     = rospy.get_param('~z_step',     0.005)
        self.err_thresh = rospy.get_param('~err_thresh', 10.0)
        self.tof_thresh = rospy.get_param('~tof_thresh', 200.0)
        rate_hz         = rospy.get_param('~rate',       30.0)

        # Perch maneuver: perch is in FRONT (+x at ToF distance), gripper on TOP
        # (+z). Go to the perch point but perch_drop below it, then climb up to
        # perch_climb ABOVE it to engage the gripper.
        self.perch_drop  = rospy.get_param('~perch_drop',  0.40)  # m below perch
        self.perch_climb = rospy.get_param('~perch_climb', 0.20)  # m above perch
        self.perch_t     = rospy.get_param('~perch_t',     3.0)   # s per traj leg

        # Tracker ignores commanded moves smaller than this; any nonzero-axis
        # correction is bumped up to at least this magnitude.
        self.min_step   = rospy.get_param('~min_step',   0.10)   # m
        # Cap each move too: gain*error is unbounded, so a large error commands a
        # huge lurch the tracker can't follow -> error never converges. Cap it
        # for a stable slow march toward center.
        self.max_step   = rospy.get_param('~max_step',   0.15)   # m
        # We publish a committed absolute setpoint at a slow fixed rate (NOT
        # every loop tick) so we never spam 10cm jumps at 30Hz.
        self.cmd_rate   = rospy.get_param('~cmd_rate',   1.0)    # Hz
        self._last_cmd  = rospy.Time(0)
        self.odom_topic = rospy.get_param('~odom_topic',
                                          '/red/vrpn_client/estimated_odometry')

        self.hold_srv   = rospy.get_param('~position_hold_srv', '/red/position_hold')
        self.do_hold    = rospy.get_param('~call_position_hold', True)

        self.state      = _first_enabled()
        self.error_x    = 0.0
        self.error_y    = 0.0
        self.tof_mm     = float('inf')
        self.odom       = None
        self.perch_done = False
        self.have_error = False   # set True once /perch/error_* arrives

        rospy.Subscriber('/perch/error_x',                      Float32,  self._cb_ex)
        rospy.Subscriber('/perch/error_y',                      Float32,  self._cb_ey)
        rospy.Subscriber('/perch/tof',                          Float32,  self._cb_tof)
        rospy.Subscriber(self.odom_topic,                       Odometry, self._cb_odom)

        self.pub_pose = rospy.Publisher(
            '/red/tracker/input_pose',       PoseStamped,                  queue_size=1)
        self.pub_traj = rospy.Publisher(
            '/red/tracker/input_trajectory', MultiDOFJointTrajectory,      queue_size=1)

        # Put the stack in position hold before we start sending setpoints.
        if self.do_hold:
            self._call_position_hold()

        rospy.Timer(rospy.Duration(1.0 / rate_hz), self._loop)

        enabled_names = [_STAGE_NAMES[s] for s in [ALIGNING, APPROACHING, PERCHING]
                         if ENABLED_STAGES.get(s, True)]
        rospy.loginfo(f"Perch controller started — enabled stages: {enabled_names} — initial state: {_STAGE_NAMES[self.state]}")

    def _call_position_hold(self):
        rospy.loginfo(f"Waiting for position-hold service {self.hold_srv} ...")
        try:
            rospy.wait_for_service(self.hold_srv, timeout=10.0)
            rospy.ServiceProxy(self.hold_srv, Empty)()
            rospy.loginfo("Position hold engaged.")
        except rospy.ROSException:
            rospy.logwarn(f"Service {self.hold_srv} not available — skipping position hold")
        except rospy.ServiceException as e:
            rospy.logwarn(f"Position-hold call failed: {e}")

    def _cb_ex(self,   msg): self.error_x = msg.data; self.have_error = True
    def _cb_ey(self,   msg): self.error_y = msg.data; self.have_error = True
    def _cb_tof(self,  msg): self.tof_mm  = msg.data
    def _cb_odom(self, msg): self.odom    = msg

    def _make_point(self, x, y, z, t=0.0):
        pt = MultiDOFJointTrajectoryPoint()
        pt.time_from_start = rospy.Duration(t)
        tf = Transform()
        tf.translation = Vector3(x=x, y=y, z=z)
        tf.rotation    = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pt.transforms  = [tf]
        return pt

    def _make_pose(self, x, y, z):
        ps = PoseStamped()
        ps.header.stamp    = rospy.Time.now()
        ps.header.frame_id = 'world'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        return ps

    def _combined_error(self):
        return math.sqrt(self.error_x ** 2 + self.error_y ** 2)

    def _clamp_step(self, d):
        """Bump a nonzero axis correction up to the tracker's minimum step.

        The tracker silently drops moves < min_step, so a small but real
        correction would otherwise do nothing. Zero stays zero (no spurious move).
        """
        if d == 0.0:
            return 0.0
        if abs(d) < self.min_step:
            return math.copysign(self.min_step, d)
        if abs(d) > self.max_step:
            return math.copysign(self.max_step, d)
        return d

    def _due_to_command(self):
        """True once per command period (cmd_rate). Gates the whole control step
        so we sample odom and add the movement exactly when we publish — one
        committed >=10cm step per second, not 30 stacked jumps."""
        now = rospy.Time.now()
        if (now - self._last_cmd).to_sec() < 1.0 / self.cmd_rate:
            return False
        self._last_cmd = now
        return True

    def _advance(self, reason):
        """Transition to the next enabled stage, logging the reason."""
        nxt = _next_enabled(self.state)
        rospy.loginfo(f"{_STAGE_NAMES[self.state]} → {_STAGE_NAMES[nxt]}  ({reason})")
        self.state = nxt

    def _loop(self, _event):
        if self.state == DONE:
            return

        # Don't act until we actually have data. Otherwise the errors are still
        # at their 0.0 init values, _combined_error() is 0 < err_thresh, and the
        # node instantly declares "aligned" and finishes without moving.
        if self.odom is None:
            rospy.logwarn_throttle(2.0, f"Waiting for odom on {self.odom_topic} ...")
            return
        if not self.have_error:
            rospy.logwarn_throttle(2.0, "Waiting for /perch/error_x,/perch/error_y ...")
            return

        pos = self.odom.pose.pose.position
        x, y, z = pos.x, pos.y, pos.z

        # ── ALIGNING ──────────────────────────────────────────────────────────
        if self.state == ALIGNING:
            err = self._combined_error()
            rospy.logdebug(f"ALIGNING  err={err:.2f}")

            if err < self.err_thresh:
                self._advance(f"aligned err={err:.2f}")
                return

            # One committed move per second, added to the CURRENT position
            # (odom is sampled in this same tick, only when due).
            if self._due_to_command():
                # Camera frame -> UAV frame:
                #   camera error_x (horizontal) -> UAV y (left/right)
                #   camera error_y (vertical)   -> UAV z (up/down)
                #   UAV x (forward) is held during alignment.
                dy = self._clamp_step(-self.gain * self.error_x)
                dz = self._clamp_step(-self.gain * self.error_y)
                self.pub_pose.publish(self._make_pose(x, y + dy, z + dz))
                rospy.loginfo(f"ALIGN cmd  dy={dy:+.3f} dz={dz:+.3f}  (from y={y:.2f} z={z:.2f})")

        # ── APPROACHING ───────────────────────────────────────────────────────
        elif self.state == APPROACHING:
            err = self._combined_error()
            rospy.logdebug(f"APPROACHING  err={err:.2f}  tof={self.tof_mm:.0f}mm")

            if err > self.err_thresh * 2.0:
                rospy.logwarn(f"Error drifted (err={err:.2f}) — back to ALIGNING")
                if ENABLED_STAGES.get(ALIGNING, True):
                    self.state = ALIGNING
                # If ALIGNING is disabled we stay in APPROACHING and keep going
                return

            if self.tof_mm < self.tof_thresh:
                self._advance(f"tof={self.tof_mm:.0f}mm < {self.tof_thresh:.0f}mm")
                return

            # One committed move per second, added to the CURRENT position.
            if self._due_to_command():
                # Aligned already — only drive FORWARD in UAV x (hold y/z).
                dx = self._clamp_step(self.z_step)   # forward approach step
                self.pub_pose.publish(self._make_pose(x + dx, y, z))
                rospy.loginfo(f"APPROACH cmd  dx={dx:+.3f}  (hold y={y:.2f} z={z:.2f})")

        # ── PERCHING ──────────────────────────────────────────────────────────
        elif self.state == PERCHING and not self.perch_done:
            # Perch is in FRONT (+x at ToF distance); gripper on TOP (+z).
            # Go to the perch point but perch_drop below it, then climb up to
            # perch_climb above it to engage. Timed legs so the tracker flies it.
            tof_m   = self.tof_mm / 1000.0
            px      = x + tof_m
            below_z = z - self.perch_drop
            engage_z = z + self.perch_climb

            traj = MultiDOFJointTrajectory()
            traj.header.stamp    = rospy.Time.now()
            traj.header.frame_id = 'world'

            traj.points.append(self._make_point(x,  y, z,        t=0.0))
            traj.points.append(self._make_point(px, y, below_z,  t=self.perch_t))
            traj.points.append(self._make_point(px, y, engage_z, t=2.0 * self.perch_t))

            self.pub_traj.publish(traj)
            self.perch_done = True
            self.state      = DONE
            rospy.loginfo(f"Perch traj: z {z:.2f} -> down {below_z:.2f} -> up {engage_z:.2f} "
                          f"(px={px:.2f}) — DONE")


def main():
    PerchController()
    rospy.spin()


if __name__ == '__main__':
    main()