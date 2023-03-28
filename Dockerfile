from alpine:latest

# install gcc, numpy, pip, protocol buffers
RUN apk add --no-cache build-base make python3-dev jpeg-dev zlib-dev libffi-dev cairo-dev pango-dev gdk-pixbuf-dev python3 py3-pip py3-numpy protobuf protoc && ln -sf python3 /usr/bin/python
RUN python3 -m ensurepip
ADD . /LDS006_git
WORKDIR /LDS006_git
RUN python3 -m venv ./venv && \
        source venv/bin/activate && \
        pip install --no-cache --upgrade pip -r requirements.txt && \
        python setup.py bdist_wheel && \
        pip install dist/lds006*.whl
RUN chmod 700 entrypoint.sh
EXPOSE 5000
ENTRYPOINT ["/LDS006_git/entrypoint.sh"]