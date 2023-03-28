#!/bin/sh
ln -s lds006/msgLDS.proto example/website/msgLDS.proto
. venv/bin/activate
exec "$@"
