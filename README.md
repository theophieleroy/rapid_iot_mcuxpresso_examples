Rapid-IoT Users,

 

Please find enclosed example projects. See below the different projects included. Here is the procedure to import those project into MCUXpresso.

This is not an SDK package, you do not want to install this package as a classical SDK.

 
#HOW TO IMPORT EXAMPLES PROJECTS

    Download RAPID-IOT-EXAMPLES package included below.
    Extract the package on your hard drive.
    Open MCUXpresso IDE.
    Import project(s) from the file system.
    Enter the path of your unpacked package, root directory option.
    Untick "Copy projects into workspace" option
    Select project(s) you want to import

 

Sources of projects can be found under the path: "boards\rapidiotk64f\demo_apps"

 
#PROJECTS PACKAGE LIST

##display_sensor

Display accelerometer information on Rapid-IoT display at regular intervals.

Learn How To :

- Instantiate a sensor

- Create a basic UI and use display

 

##display_sensor_touch_interface

Display gyroscope and accelerometer information on Rapid-IoT display at regular intervals.

Learn How To:

- Instantiate sensor

- Create a basic UI and use display

- Manage interrupts linked to button and touch

 

##ncf_tag_write

Record time stamps for falls and shocks in the NFC tags. The use case is logistic/asset tracking.

Learn How To:

- Use the NFC tag to store information

 
##ble_sensors

Display temperature and humidity information collected by kit and send them over BLE to a smartphone.

Learn How To:

- Join a BLE network from the kit point of view

 

##ble_led_control

Change LED state color and intensity over BLE

Learn How To:

- Activate kit features via BLE

 

##weather_station

Display temperature, humidity, air pressure and light information collected on kit on a mobile phone via BLE.

Learn How To:

- Develop a full product from IoT to cloud including security

- Have a view on the cloud mobile app. Code

- Have a view on the cloud and web UI related codes.
