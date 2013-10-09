cp ~/sketchbook/burner_ino/.build/diecimila-55156a30/firmware.hex .
avrdude -p m168 -c usbtiny -B 1 -U flash:w:firmware.hex

