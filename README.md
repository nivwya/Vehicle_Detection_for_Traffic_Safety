# Vehicle_Detection_for_Traffic_Safety
This project detects all types of vehicles and give a warning signal if the vehicle is overspeeding, if motorbike rider is not wearing a helmet, or if more than 2 people ride motorbike, and if cyclist is not holding the handle.

Check it out at: https://www.youtube.com/watch?v=zE-AC-NhYtw&list=LL&index=1&t=2s

Steps to Run The Agent from Scratch

1. Install Python:
   - Download and install Python 3.8 or later

2.. Install Required Libraries:
   - Run the following command:
    pip install opencv-python
    pip install numpy
    pip install pyserial
   pip install ultralytics
   pip install deep-sort-realtime

4. Set Up Your Arduino:
   - Open Arduino IDE.
   - Connect the Arduino UNO via USB.
   - Upload the Arduino code to the Arduino

5. Prepare the LCD Display :
   - Connect the LCD 20x4 with I2C module to Arduino:
     GND -> GND
     VCC -> 5V
     SDA -> A4
     SCL -> A5

6. Run the Python Code:
   - Open the main Python script.

7. The LCD will display the messages according to what is captured by the camera
