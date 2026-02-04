#!/bin/bash
set -e
cd ~/projects/TRC
./run_pi.sh 2>&1 | tee -a logs/run_$(date +%Y%m%d_%H%M%S).log
