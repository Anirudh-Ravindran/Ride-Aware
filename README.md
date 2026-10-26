Inspiration
As a group who all ride electric scooter to commute to class, we notice key problems with these vehicles and similar ones in our community and campus. Problems such as many people getting tickets, a increase in the amount of scooter/bike crashes, and miscommunication on the road between cars to bikes and scooters can be seen almost everyday. Observing these challenges in our day to day lives embarked us to create a change.

What it does
RideAware is universal smart safety attachment that provides turn signals LED's on the rear and the ends the handlebars, a break light system that runs on a IMU sensor's deceleration data, and a rear distance monitor that alarms the user when a vehicles gets to close and provides the exact amount in a OLED screen on the handlebar. All of these functions combined provide the driver more awareness on the road to decrease and prevent the problems related to bikes and scooters.

How we built it
We started making our product with the creation of two separate components, one on the handle bars and one the back. The two communicate with each other with ESP-NOW two way communication to transfer inputs and outputs. The handle bar component contains the OLED screen, 2 LED's on the ends of each handle bar, and two buttons providing left and right turn signal inputs. We built this for module on a breadboard and attached it to one of our scooters. The main wiring, breadboard, and ESP32 sit in a red cup tapped to the stem of the scooter. The component in the back has a ultrasonic sensor, ESP32, 2 LED's for turn signals, 1 LED for break lights, a IMU sensor for movement data, and a power module to power the ultrasonic sensor. We used a small breadboard in the back and attached to the heel of the scooter platform. A casing made out of cardboard was attached to protect the wiring and components.

Challenges we ran into
Some challenges we ran into were figuring out wireless communication between two separate components, fine tuning the IMU sensor, figuring out how to attach our product onto the scooter, and numerous ESP32 port issues.

Accomplishments that we're proud of
Some accomplishment that we are proud of are pivoting from a amplifier with a speaker to audio being relayed from a phone when connected to the ESP32 network. Additionally, when connected to the network you are brought to a web app that allows you to adjust the volume of the warning and turn signal sounds while also being able to mute the warnings. You can also see the distance of any vehicles behind you show up in the web app. This means people who are wearing headphones connected to their phone will still be able to the hear the warnings.

What we learned
This was the first time for all of us on using ESP32 wireless network communications. Additionally, this was all of our first times doing a hackathon, being a hardware one at that, and we feel like we learned how to work under pressure and in a short amount of time and still create a effective and meaningful project.

What's next for RideAware
Some feature improvements that we could make to our product is analyzing the data further in combination with a GPS component to find exactly where most unsafe driving occurs. With this information users, city planners, and college transportation departments can make changes to relieve these problems. Additionally, making it weather resistant casing will allow it to be used anywhere and anytime. Lastly, we believe that there is a lot of spaces where we can optimize the actual hardware, an example of this could be to move to a custom PCB to cut down on wiring and save space.

