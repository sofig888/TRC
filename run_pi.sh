#!/bin/bash
set -e

cd ~/projects/TRC
git pull

source .venv/bin/activate

# Cambia esta línea por el comando real de tu proyecto si es distinto:
python3 main.py
