# hendictrl
Hendi plugin for craftbeerpi3. This package contains several plugins:
- PIDHendi, a KettleController with PID temperature control
- BoilHendi, a KettleController for powwr control
- HendiControl, a Actor for Hendi induction coocker
- ToBoilStep, a step for taking the wort to boil
- BoilStep, a step for boiling the wort
# Installation
Download via CraftBeerPi Web Interface and restart CraftBeerPi.
# Setup
1.  Create a sensor, for example "mash temperature"
1.  Create an actor, for example "Hendi"
    1. configure "Max Power" to 100 (or a suitable value for your equipment)
    1. configure "PWM frequency" to 100 (or a suitable value for your equipment)
    1. configure "Power control GPIO" to 27 (or a suitable value for your equipment)
    1. configure "On/Off control GPIO" to 22 (or a suitable value for your equipment)
1.  Create a (mash)kettle for temperature control, for example "Mash"
    1. configure "Logic" to PIDHendi
    1. select "Actor" -> Hendi (or the name choosen i §3)
    1. select "Agitator" if applicable 
    1. select "Sensor" -> mash temperature (or the name choosen i §2)   
    1. configure "D" to 0 (or a suitable value for your equipment)
    1. configure "I" to 40 (or a suitable value for your equipment)
    1. configure "P" to 140 (or a suitable value for your equipment)
    1. configure "Max Power" to 100, will be evaluated and might disapear in the future
1.  Create a (boil)kettle for power control, for example "Boil"
    1. select "Logic" -> BoilHendi
    1. select "Actor" -> Hendi (or the name choosen i §3)
    1. select "Agitator" if applicable
    1.  select "Sensor" -> mash temperature (or the name choosen i §2)
    
# More information
https://github.com/AHm74/hendictrl/wiki

