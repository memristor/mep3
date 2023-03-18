This hardware interface was initialy forked from [dynamixel_hardware](https://github.com/dynamixel-community/dynamixel_hardware).
However, over time, we introduced changes, so it is very hard for us to merge with the upstream.

Some changes that we introduced:
- Added support for v1 protocol.
- Set speed at each update, so we can do position+velocity control.
- Added multithreading to avoid long reads and writes.
- Added update_rate parameter to throttle read/write if necessary.
