#!/bin/sh

! [ -f /memristor/.setup/completed ] && \
cd /memristor/.setup && make "provision-$PLATFORM"
cd && bash
