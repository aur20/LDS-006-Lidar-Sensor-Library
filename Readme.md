# LDS-006-Lidar-Sensor-Library

Greetings programs, [this](https://www.jentsch.io/lds-006-lidar-sensor-reverse-engineering/) is a high quality article about the sensor I recently found in an old vacuum cleaner. Sadly, from software side of things it is a stub.

So I brought some upgrades. To conduct tests I am using Raspberry Pi Model 3/4. Feel free to use the library with any other hardware.

## How to connect hardware

```
black: GND
red: VCC (5V)
blue: UART TX (3v3, Pin 10 @ RPI)
green: UART RX (3v3, Pin 8 @ RPI)
```

## How to build library

In this directory: setup a virtual environment, then install library
```
sudo apt-get install python3-venv
python3 -m venv ./venv
source venv/bin/activate
pip install -r requirements.txt
python setup.py bdist_wheel
pip install dist/lds006*.whl
ln lds006/msgLDS.proto example/website/msgLDS.proto
```
**Or** use the Dockerfile
```
docker build -t lidar .
docker run -it --device=/dev/serial0 -p5000:5000 lidar sh
python3 examples/web/main.py /dev/serial0
```

## How to use  library

In this directory: start virtual environment, then run 'main.py'.

**Caution: requires access to hardware** Best practice is to add user to group *dialout*.

I would like to introduce a clean interface, thus use [Protocol Buffers](https://protobuf.dev/getting-started/pythontutorial/). Any subscriber message is returned via message encoding specified in `msgLds.proto`.

Example can be run by:

```
. venv/bin/activate
python examples/simple.py /dev/serial0
python examples/callback.py /dev/serial0
python examples/website/main.py /dev/serial0
```

## Thougts on used software

The library will open serial device from the passed string. This is done by [pyserial](https://pythonhosted.org/pyserial/) from within the class. The library will also close serial device when destroying manager object. At all, such behaviour is not best practice but in the scope of this project I could not inherit from `serial.Serial`. Passing an object to the library would be better but who is responsible to close it?

In the latest patch I completely removed filtering (so NumPy is not used). Filtering data seems to be not required. I reverted to distinguish good from bad data by using the `reflectivity`-factor. Still, there appear runaway data points, which are marked invalid if they exceed standard deviation.

Website example includes [protobuf.js](https://github.com/protobufjs/protobuf.js/) v7.2.2. For my environment is not always connected to the internet, I included source code in this repository.