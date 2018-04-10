Compile using older board definition and older RFID library.

Instructions:

Download the Arduino IDE for your system: https://www.arduino.cc/en/Main/Software

Clone the Github directory or just download the zip file from the repository. When you have the files put them in your arduino sketchbook folder (You can view your preferences in the Arduino IDE to see what this folder is), or just put the folder somewhere where you can find it. 

Now the hard part. The customized circuit board needs a customized board definition for it to work with the Arduio software. The board definition indicates how the pins are numbered and what each pin can do (input/output, analog-to-digital conversion, external interrupt, etc). To install a new board definition you should……??????

Now open the file that ends with .ino in the Arduino IDE. Select the newly installed board definition (Tools -> Board -> …). Then turn the board on. Select the correct port (Tools -> Port -> …).   

You should now be able to compile and upload the code to the Board. It will begin executing the code as soon as it is done uploading. Use Tools -> Serial Monitor to communicate with the circuit board. You have to open the serial monitor window quickly because the reader automatically starts logging data after a few seconds. If you miss the time window or communication stops, turn the board off and on and then close and reopen the serial monitor window.