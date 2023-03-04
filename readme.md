Readme
======
Warning 1
---------
This software is without *any* warranty or guarantees, or worse. It may set your house on fire and kill your family. It is not finished, not tested and cannot be relied on to work.

Warning 2
---------
The BIC2200 that's used as an inverter is not sufficiently certified for grid-tied usage in Europe. So you can only use this system as a grid-following device while in island-operation.

Introduction
------------
BESSer is a home battery storage system. It uses a Meanwell BIC as a charger/inverter. As for batteries, it communicates using the PylonTech CAN protocol. 

Integration with external services is provided through MQTT. Home assistant example code for integration is in development. 

Hardware requirements
---------------------
Due to a can speed mismatch two can busses are required by default, as the pylontech and the BIC cannot communicate at the same speed. 

Installation
------------
First copy over ```config.py.example``` to ```config.py```. Edit it to be in accordance with your configuration.

Then execute 

```pipenv install```

to install dependencies. Run ```pipenv shell``` to get a shell where your dependencies are present then execute

```python main.py``` 

to start running. 


Credits
-------
- https://github.com/PaulSturbo/DIY-BMS-CAN/blob/main/SEPLOS%20BMS%20CAN%20Protocoll%20V1.0.pdf
- https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication
