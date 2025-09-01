# NASA-Glenn-Digital-Twin-Testbed
This project focuses on creating a simple model of a Digital Twin for an electric motor. It will be used to educate others on what a Digital Twin is. A Digital Twin is a digital representation of a physical system. Using various sensors on the physical system, data can be collected and then sent to the Digital Twin. This is so that the Digital Twin can “copy” the data and be used to run various simulations. The data from the simulations can then be applied to the physical system. The Digital Twin is designed to have a two-way flow of information so that it is constantly adapting and changing based on the data it gets from the sensors.

Digital Twins originated from NASA during the Apollo mission. After the Apollo 13’s oxygen tank explosion that caused damage to the main engine, NASA created a “living model” of the vehicle. This was to predict any possible failure to prevent any future harm. Digital Twins are now needed for physical systems that are too large in scale to be tested physically, hence the Digital Twin.
Other rising uses:
• Jet engine health management
• Self tuning Digital Twin technology

System Characterization
𝑅 =
𝑉
𝐼
Where:
R = Motor Winding Resistance
I = Motor Winding Current
V = Voltage
Under stall conditions.
𝐾𝑣 =
𝑉 − 𝐼𝑅
𝜔
Where:
Kv= Motor Voltage Coefficient
𝜔 = Motor Shaft Speed
Mechanical Hardware
Physical System Parts:
• Motor to be characterized
• Arduino Mega
• Digital Temperature Sensor
• Accelerometer
• Sound Sensor
• Current Sensor
• Brake Motor

1. The Arduino Code reads the raw values by calling
functions
2. The data is then sent to the laptop where python
and bokeh creates a plot of calculated engineering
values that can be analyzed
3. Signal processing used to clean noisy data
