#!/bin/bash

set -e  # Exit on error

echo "Uploading files to XIAO nRF52840..."

mpremote cp code.py boot.py :

echo "✓ Upload complete!"
