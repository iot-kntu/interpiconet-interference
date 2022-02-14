# Inter-piconet Interference Mitigation

## Abstract

With the growth of the Internet of Things and the use of various communication devices in real environments, the use of network resources is becoming more and more important. IoT communication protocols operate in the 2.4 GHz frequency band, which can be used without a license. In this frequency band, communication protocols such as Bluetooth Low Energy (BLE), Wi-Fi and ZigBee are used by different devices. On a large scale, with the increase in the number of devices that use this frequency band, the optimal use of this frequency band is considered. BLE protocol is one of the main communication protocols used in IoT. In crowded environments, several BLE devices may form piconets and exchange data when connected. In BLE, adaptive frequency hopping (AFH) is used to deal with interference. This anti-interference method is very effective for BLE network interference with other networks, but interference between different piconets still exists. In this research, solutions to deal with inter-piconet interference in BLE have been proposed using a central controller. In this research, channel removal (CR) algorithm and channel separation (CS) algorithm are introduced to reduce interference between piconets. The evaluation of the proposed solutions has been done in the form of simulation and practical implementation. In this study, we have seen a reduction in interference between BLE piconets and better use of network resources.

## Simulation

We implemented simulation in python. first run the main.py file in simulation folder then run plot.py for charts.

## Controller

This is for controller node. when you run main.py in experimental folder, the master node scans and collects channel maps then runs channel separation algorithm and advertises new channel maps.

## Master

Put ble_app_multirole_lesc_interference_mitigation folder in nRF5_SDK_17.0.2/examples/ble_central_and_peripheral/experimental and run the .em file with SEGGER Embedded Studio.