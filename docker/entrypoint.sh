#!/bin/sh

cd /memristor/.setup && make "provision-$PLATFORM"
cd && bash
