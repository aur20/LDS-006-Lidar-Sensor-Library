#!/bin/sh
ln lds006/msgLDS.proto example/website/msgLDS.proto
. venv/bin/activate
exec "$@"
