# Contact and TF qualification

**Status: qualified.** All three fixed-seed cases passed in report
`dbfba035-4709-4916-8093-7765b1347403`.

The ground-only case reported zero contact events. The cone case identified
`cone_01`, and the multi-fixture case independently identified both
`cone_01` and `pedestrian_01`. For every captured sensor timestamp, the checker
looked up `camera_link`, `imu_link`, and `lidar_link` in TF; all lookups
resolved with zero gaps (246–282 samples per run).

~~~bash
python3 tools/rc_module_matrix.py --module contact_tf
~~~
