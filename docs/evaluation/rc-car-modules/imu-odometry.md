# IMU and odometry qualification

**Status: qualified.** All four fixed-seed cases passed in report
`6813ba88-6e7a-4dc0-a430-af336a7c173c`.

- At rest: 100.15 Hz from 397 samples, strictly increasing simulation stamps;
  peak angular-rate magnitude was `1.15e-9 rad/s`, mean vertical acceleration
  was `9.8 m/s²`, and measured odometry drift was below `3e-19 m`.
- Straight line: 100.18 Hz; integrated wheel-odometry distance differed from
  the commanded profile by `0.00994 m`.
- Turning: 100 Hz; yaw differed from the configured bicycle-model prediction
  by `0.1133 rad`, within the frozen tolerance.
- Pause and clock reset: one timestamp epoch reset was observed, and IMU and
  odometry streams resumed afterward.

These bounds characterize the configured simulated sensor noise model and
wheel-odometry behavior. The SDF defines no explicit IMU noise, so the near-zero
rest noise and drift are simulation observations, not claims about real IMU
accuracy.

~~~bash
python3 tools/rc_module_matrix.py --module imu_odometry
~~~
