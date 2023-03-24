# Dummy branch for development

In this branch, no sensor functionality is used. Instead, data is randomly generated and sent to the server.

Code is in `example/canvas_drawing`

I compiled message scheme to send data from server to website wih the following command:
'''
    protoc --python_out=. msgLDS.proto 
'''
using a recent version of protoc*: 

'''
    protoc --version
    libprotoc 3.21.1
'''

*Default installation of ROS-noetic uses version 3.6.1 which is too old to work with recent python library version.

...