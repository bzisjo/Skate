# Skate
Skate Form Tracker

#AtmelStudio Setup for USBTinyISP + AVRDUDE
Download AVRDUDE from [here](http://www.nongnu.org/avrdude/). In AtmelStudio, go to Tools -> External Tools, and setup a new Menu content with the following parameterse:  
__Title:__ USBTiny ATmega328p  
__Command:__ (dir of AVRDUDE binary)\avrdude.exe  
__Argumnets:__ -c usbtiny -p atmega328p -U flash:w:$(TargetDir)$(TargetName).hex:i -C avrdude.conf -v  
__Initial Directory:__ (dir of AVRDUDE binary)  
  
Check the "Use Output window" option, and click OK.
