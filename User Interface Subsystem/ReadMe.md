Modified the Object Detection subsystem to make a functional GUI for practical use.

Added Functionality:
  variable Refill will be acessible via http get messages from the gui. Every 5 seconds it will get the value of Refill. 

  variable refill is initialized to False, will become True if the solenoid valve has activated and deactivated 50 times. 
  50 times is an estimate of how many sprays will empty the reservoir enough to reduce pressure in the deterrent subsystem. 
