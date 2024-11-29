# arduinoHackathon
 
To finally conclude and get this project documented, this is the final code of the [Brose excellence program](https://www.linkedin.com/posts/brose-srbija_brose-laboratorija-activity-7267548457840599040-5iwr?utm_source=share&utm_medium=member_desktop) challenge.
It was a 2 day hackathon spanning 7 hours on one day and 4 on another so in total, 11 hours of intense work. To be honest even though we only managed to acquire 2nd place, I'm still very proud of my team.
The competition was on making an autonomous vehicle of sorts with the following tasks:
## Summary of Tasks

1. **IR Remote Control Functionality [5 Points]**  
   Implement remote control functionality using an IR remote. The car must support forward/backward movement, in-place left/right rotation, and stopping. Actions are mutually exclusive, with the last command taking precedence. Pressing the same button again stops the car, and a dedicated stop button sets the car to idle.

2. **Optical Encoder [15 Points]**  
   - **Distance Measurement [5 Points]:**  
     Use an optical encoder to estimate the distance traveled, regardless of direction. Pressing button "1" starts/stops distance measurement, with results displayed via serial communication. Accuracy within ±15% of actual distance yields full points.  
   - **Maintaining Constant RPM [10 Points]:**  
     Implement a digital controller to maintain a set RPM. Button "2" starts the vehicle and regulates the speed based on a software-defined variable. Real-time RPM data must be transmitted via serial communication at least 5 times per second.

3. **Ultrasonic Distance Sensor [25 Points]**  
   - **Automatic Stopping [5 Points]:**  
     Enable automatic stopping in front of obstacles, aiming for minimal distance without collision. Each 1 cm away from the obstacle deducts a point. Three attempts are allowed, with the best result used for scoring. Collisions incur additional penalties.  
   - **Obstacle Following [20 Points]:**  
     The car maintains a constant distance of 15 cm from an obstacle using an ultrasonic sensor. Button "6" toggles this functionality. Average distance from the obstacle is used for scoring, with regular error and mean error calculations every 100 ms, logged via serial communication.

4. **Final Presentation [5 Points]**  
   Create and deliver a 15-minute PowerPoint presentation in English using the provided template. The presentation must showcase solutions, challenges, and potential future steps, with emphasis on clarity and professional delivery.

   We completed tasks 1,3,4 all the way with some issues on the 2nd tasks seeing as the documentation on the internet was scarce and the task wasn't even that well defined. Nevertheless we managed to score points and come out on top.

![Autonomous Vehicle](./rory.jpeg)
There she is. I think with 2 more hours, we would've gotten max points, but hey, can't win em all today