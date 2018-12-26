# UI Manager Guide

The UI manager is loaded from app_init during the main_task. The user will see a black screen with
white writing signaling the UI manager is ready to use.

Once the UI manager is ready, the user can use the HMI features to interact with the Rapid-IoT device.

The following HMI features are available:

* Momentary push buttons
    - SW1
    - SW2
    - SW3
    - SW4

* Capacitive touch pads
    - Touch Up
    - Touch Down
    - Touch Right
    - Touch Left

Touching one of these will generate a unique change on the display through the Rapid-IoT's GUI.

The expected behaviors are:

* Touch Up
    - Display a blue arrow pointing up

* Touch Down
    - Display a yellow arrow pointing down

* Touch Right
    - Display a green arrow pointing right

* Touch Left
    - Display a red arrow pointing left

* SW1
    - Display white box with text indicating sw1 button press count since boot

* SW2
    - Display white box with text indicating sw2 button press count since boot

* SW3
    - Display white box with text indicating sw3 button press count since boot

* SW4
    - Display white box with text indicating sw4 button press count since boot
