from gpiozero import DigitalOutputDevice 

RELAY_PIN = 27
ACTIVE_LOW = True

relay = DigitalOutputDevice(RELAY_PIN, active_high=not ACTIVE_LOW, initial_value=False)


def cleaning_on():
    relay.on() 


def cleaning_off():
    relay.off()


def cleanup():
    cleaning_off()
    relay.close()
