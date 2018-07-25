#!/bin/sh
hexdump -C "$1" | cut -b9- | cut -d"|" -f1 | tr -d ' \t\n\r'
