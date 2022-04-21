python-i2c-lcd
==============

### Setup

```
sudo apt-get install python-smbus i2c-tools
sudo modprobe i2c-dev
sudo modprobe i2c-bcm2708
```

add `i2c-dev` to `/etc/modules`.


### Usage

```
python2 ./display.py line_1~line_2~line_3~line_4
```

or clock demo:


```
python2 ./datetime-test.py
```

### Based on
*  https://github.com/pimatic/pimatic/issues/271
*  http://www.gejanssen.com/howto/i2c_display_2004_raspberrypi/index.html