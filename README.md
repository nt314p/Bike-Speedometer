# Bike-Speedometer
A bluetooth bike speedometer and odometer based on a HM-11/CC2541

### How does it work?

The speedometer has two parts: a wheel mounted magnet, and the main speedometer device mounted on the wheel fork. Inside the speedometer is a reed switch, which detects when the magnet on the wheel passes by. This is how the speedometer counts revolutions.

The distance the bike has traveled is derived from the revolution count. The speed the bike is traveling at can be calculated from the time it takes one revolution.

After connecting to the speedometer over bluetooth, the distance and speed data is periodically sent to the receiving device, which can then display the data. The receiving device must also have knowledge of the wheel circumference to convert wheel revolutions to distance. For example, one revolution of a wheel with a 2 meter circumference means the bike moves 2 meters every revolution of the wheel.
