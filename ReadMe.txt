Compile using older board definition and older RFID library.

Instructions:

Download the Arduino IDE for your system: https://www.arduino.cc/en/Main/Software

Clone the Github directory or just download the zip file from the repository. When you have the files put them in your arduino sketchbook folder (You can view your preferences in the Arduino IDE to see what this folder is), or just put the folder somewhere where you can find it. 

Now the hard part. The customized circuit board needs a customized board definition for it to work with the Arduio software. The board definition indicates how the pins are numbered and what each pin can do (input/output, analog-to-digital conversion, external interrupt, etc). To install a new board definition you must do two things

1) Unzip “etagrifdold.zip” and put the whole folder in the correct directory. For me the targe directory is:
/USERS/[username]/Library/Arduio15/packages/arduion/hardware/samd/16.15/variants.
I don’t know what it will be for other systems and Arduino versions.

2) Find the text file called boards.txt located in one or two directories above the one used above. For me it is:
/USERS/[username]/Library/Arduio15/packages/arduion/hardware/samd/16.15
open the text file and append the text in “add_to_boards.txt.” Then save and close. 

Now open the file that ends with .ino in the Arduino IDE. Select the newly installed board definition (Tools -> Board -> ETAG OLD RIFD). Then turn the board on. Select the correct port (Tools -> Port -> …).   

You should now be able to compile and upload the code to the Board. It will begin executing the code as soon as it is done uploading. Use Tools -> Serial Monitor to communicate with the circuit board. You have to open the serial monitor window quickly because the reader automatically starts logging data after a few seconds. If you miss the time window or communication stops, turn the board off and on and then close and reopen the serial monitor window.