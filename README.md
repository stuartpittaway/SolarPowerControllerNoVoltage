SolarPowerControllerNoVoltage.ino

Stuart Pittaway, Sept 2012.

Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)
http://creativecommons.org/licenses/by-nc-sa/3.0/
You are free:
to Share — to copy, distribute and transmit the work
to Remix — to adapt the work
Under the following conditions:
Attribution — You must attribute the work in the manner specified by the author or licensor (but not in any way that suggests that they endorse you or your use of the work).
Noncommercial — You may not use this work for commercial purposes.
Share Alike — If you alter, transform, or build upon this work, you may distribute the resulting work only under the same or similar license to this one.


---------------------------------------
This is an attempt to test out a theory that a synthetic sine wave can be used for solar power diversion
in a similar fashion to Robin Emley's design (calypso_rae on Open Energy Monitor Forum July 2012).

Inspiration by pmcalli (Open Energy Monitor Forum Sept 2012).

The circuit is using a zero cross detector (trigger on positive volt crossing) on 
the voltage waveform (connected to INT1, PIN 4 on CPU).

If this design works, solar PV diversion could be made without expensive (or accurate) transformers.

You can ignore the OPAMP GAIN sections from the code below if your not using any Opamp circuts.
---------------------------------------