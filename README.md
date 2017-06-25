# hendictrl
Hendi plugin for craftbeerpi3 

1. Install plugin from cbi addin service
2. Create a sensor, for example "mash temperature"
3. Create an actor, for example "Hendi"

    a. configure "Max Power" to 100 (or a suitable value for your equipment)
    
    b. configure "PWM frequency" to 100 (or a suitable value for your equipment)
    
    c. configure "Power control GPIO" to 27 (or a suitable value for your equipment)
    
    d. configure "On/Off control GPIO" to 22 (or a suitable value for your equipment)
    
4. Create a (mash)kettle for temperature control, for example "Mash"
    configure "Logic" to PIDHendi
    select "Actor" -> Hendi (or the name choosen i §3)
    select "Agitator" if applicable
    select "Sensor" -> mash temperature (or the name choosen i §2)
    configure "D" to 0 (or a suitable value for your equipment)
    configure "I" to 40 (or a suitable value for your equipment)
    configure "P" to 140 (or a suitable value for your equipment)
    configure "Max Power" to 100, will be evaluated and might disapear in the future
5. Create a (boil)kettle for power control, for example "Boil"
    select "Logic" -> BoilHendi
    select "Actor" -> Hendi (or the name choosen i §3)
    select "Agitator" if applicable
    select "Sensor" -> mash temperature (or the name choosen i §2)

