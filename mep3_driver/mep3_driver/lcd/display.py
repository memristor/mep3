from lcddriver import lcd
import sys

lines = str(sys.argv[1])

if lines == "off" :
  my_lcd = lcd()
  my_lcd.backlight_off();
  quit()

if lines == "on" :
  my_lcd = lcd()
  my_lcd.backlight_on();
  quit()

  my_lcd = lcd()
  my_lcd.clear()

lines = lines.split('~')
my_lcd = lcd()
for (i, line) in enumerate(lines):
  line = line.replace('_', ' ')
  if len(line) > 16 :
    line = lines[0:16]
  my_lcd.display_string(line, i+1)