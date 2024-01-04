#!/bin/bash

round_idx=0
total_round=10

while (( $round_idx < $total_round )); do
    python test_multiple_scenes_FWcheck.py

((round_idx=$round_idx+1));
done