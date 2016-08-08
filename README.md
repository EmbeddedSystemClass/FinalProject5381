#FinalProject5381


My final project for UC Santa Cruz Silicon Valley Extension course 5381.037 (Intro to Real-Time Embedded Systems Programming) with instructor Anil Gathala, Summer 2016 term.

##File Layout
The Example01 folder contains my project source.
Other folders are generated for all LPCOpen projects and are rarely modified, except for the file FreeRTOSConfig.h.

##Project Specifications

This final project allows us to bring together what we've learned in the course so far.
Although 3 options were given, I'll be attempting them all.

##Features (Common)

- Implement a full Smart Bulb application in FreeRTOS
- The LED on-board is the Bulb. In addition use the push button and light sensor.

###Level A: (10 marks)
- Periodically poll the light sensor
- When dark – light up the bulb, otherwise light off the bulb
- **(Custom)** I added support for the 8 RGB colors and display them according to light intensity.

###Level B: (10 marks)
- Add a push button
- Pressing a push button -> switch on light anyway (ignore the light sensor for now)
- Pressing the push button second time -> switch off light and go back to the smart bulb mode

##Features (Option 1)
###Level C (Bonus: +5 marks)
- Add a seven segment display (use gpio to control each led inside 7 segment)
- This should display the level of light as seen on the light sensor
- Choose a simple scale of your choice: example 0 -> dark; 8 -> max light.

##Features (Option 2)
###Level C (Bonus: +5 marks)
- Add a PIR occupancy sensor (GPIO)
- Link: https://www.adafruit.com/products/189
- Make smart bulb decisions based on both light as well as occupancy

##Features (Option 3)
###Level C (Bonus: +10 marks)
- Add a I2C/SPI based temp/hum sensor
- Whenever light state changes (on -> off; off -> on), display both temp and hum

## Project Submission Guidelines
- Submit just one zip file – with all the project files. I should be able to import it and run succesfully.
- Make a simple/quick video of the working demo from you phone and submit the video
- A simple / quick document of the s/w design, a block diagram or a flow diagram showing any of the following components implemented:
- Tasks
- 2. message queues
- ISRs and bottom-halves
- Timers
- Semaphores, etc. used
- Also show the data flow between components
- - Please also write a few lines in the text:
- At what reading of the light sensor your code lights up/off the bulb (thresholds)
- If doing bonus: mention the 7-segment display scale
- Nothing fancy:
- Make sure your video has enough light
- The document should be ideally in doc, but can be on a white sheet and scan it.
- Deadline: Sep 1st is the deadline (11:59 pm)
- Submit everything by then
