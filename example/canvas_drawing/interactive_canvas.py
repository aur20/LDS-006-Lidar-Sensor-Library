#!/usr/bin/env python3
import sys, json, flask, math
from queue import Queue
from time import sleep
import random as Random
import msgLDS_pb2

app = flask.Flask(__name__, template_folder=".")

# load main page
@app.route('/')
def index():
    return flask.render_template("index.html")

# load protobuf definitions
@app.route('/msgLDS.proto')
def pb():
    return flask.render_template("msgLDS.proto")

# load protobuf.js sources
@app.route('/protobuf-7.2.2.js')
def src_protobuf():
    return flask.render_template("protobuf-7.2.2.js")
# load protobuf.js.map sources
@app.route('/protobuf.js.map')
def src_protobuf2():
    return flask.render_template("protobuf.js.map")
# load update page
@app.route('/lidar')
def chart_data(): 
    def generate_data():
        while True:
            msg = msgLDS_pb2.msgLDS()
            data = [math.floor(Random.random()*20000) for i in range(0, 360)]
            for i in range(360):
                _msg = msg.data.add()
                _msg.angle = i
                _msg.distance = max(3000, data[i])
                _msg.certainty = False
                if Random.randint(0, 1) == 1:
                    _msg.certainty = True

            data = msg.SerializeToString()
            #print(data)
            #print("Length of data: " + str(len(data)))
            yield f"data:{data}\n\n"
            #json_data = json.dumps({'values': data })
            #yield f"data:{json_data}\n\n"
            sleep(3)
    
    response = flask.Response(flask.stream_with_context(generate_data()), mimetype="text/event-stream")
    response.headers["Cache-Control"] = "no-cache"
    response.headers["X-Accel-Buffering"] = "no"
    return response

@app.route('/start')
def start_sensor():
        
    return "\n\n"
@app.route('/pause')
def pause_sensor():
        
    return "\n\n"
@app.route('/stop')
def stop_sensor():
        
    return "\n\n"

if __name__ == "__main__":
    app.run(debug=True, threaded=True)