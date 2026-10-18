# Forward-facing phone mount calibration

This procedure assumes the iPhone rear camera is mounted facing the
rover's forward direction. ARKit tracks visible room features while 
the rover turns.

## Record measured mount geometry

- [ ] Mark the rover reference point (recommended:
      vertical projection of the midpoint between drive wheels).
- [ ] Optionally measure rear camera forward and left offsets as initial estimates;
      the two-way turn fit estimates these horizontal offsets.
- [ ] Measure rear camera optical-center height from ground plane.
- [ ] Measure camera roll, pitch, and yaw relative to rover axes 
      if needed, or else 0, in degrees.
- [ ] Enter measurements in the app. Its automatic turn fit replaces the starting
      forward/left offsets after the two turn passes agree.
- [ ] Corrections are applied through the app relative to upright rear-camera-forward mounting:
      rover +X forward, +Y left, +Z up; roll X, pitch Y, yaw Z.
- [ ] Confirm phone and mount are static relative to the rover.

## Phone-assisted in-place-turn check

1. Put the rover on a level, non-slippery surface in a well-lit area with
   textured features visible to the rear camera. Keep people and objects clear of wheels.
2. Start **Phone IMU + VIO** on the phone. Enter and save a plausible measured
   camera height before starting the offset fit.
3. Keep motors disconnected or disabled. Tap **Start offset turn pass**, then
   slowly turn the rover at least 90 degrees in one direction around the marked
   rover reference point. Avoid lifting or translating the chassis. Tap
   **Finish turn pass**.
4. If the first pass is accepted, tap **Start offset turn pass** again and turn
   at least 90 degrees in the opposite direction around the same point. Finish
   the second pass.
5. The app fits the horizontal camera offset from ARKit camera positions and
   rover heading. It requires at least 30 samples over 2 seconds, at least
   90 degrees of net turn, a path RMS error no greater than 5 cm per pass, and
   reverse-pass offset estimates within 5 cm. Tracking loss cancels the active
   pass. Rejected fits require repeating the turns with a steady pivot and
   reliable tracking.
6. Review the reported forward/left offset and pass RMS errors. Tap
   **Use fitted offset**, then **Save measured mount calibration**. The fit
   does not estimate camera height or mount-angle corrections; those remain
   measured inputs. Record the result and reviewer.

## Sensor and ground checks

- [ ] Stationary IMU acceleration and gyro are stable and axes have expected signs.
- [ ] Short straight push reports forward motion with little lateral velocity.
- [ ] In-place turn reports rotation without fictitious rover-origin translation.
- [ ] Depth map ground line/returns agree with the measured ground plane and height.
- [ ] Tracking loss, interruption, and relocalization produce a safe stop/reset.
- [ ] Lever-arm correction `v_rover = v_camera - omega × r_camera` uses one frame
      consistently and is checked during the in-place turn.
- [ ] The saved calibration shows the fitted horizontal offset and manually
      measured height and mount-angle corrections.

## Review gate

- [ ] Calibration values are measured, plausible, saved, and reviewed.
- [ ] Real-rover controller gains and limits are tuned and recorded separately.
- [ ] Motor adapter stays disabled until calibration and tuning are signed off.

ARKit world tracking is local visual-inertial tracking, not a surveyed global
reference. Repeat the check after changing the phone, mount, or mount position.
