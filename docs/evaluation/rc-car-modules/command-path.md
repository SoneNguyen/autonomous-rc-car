# Command path matrix (stretch)

**Status: qualified.** The optional three-case set passed in report
`a5ef0659-a54e-4e0b-96bd-fee849fe7b09`.

The live controller clamped the over-limit Ackermann request to its existing
configured speed and yaw bounds. The observed step-response latency was
`0.103 s`. The watchdog case remained active at `0.397 s` and reported
`safety_watchdog_stop` at `0.598 s` around the existing `0.5 s` lease.
The checker reads the controller and vehicle limits without changing them.

~~~bash
python3 tools/rc_module_matrix.py --module command_path --include-stretch
~~~
