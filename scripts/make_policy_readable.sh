#! /bin/bash

PROJECT_DIR = $(pwd)

cd config/PDDL
python2 ../.././planner-for-relevant-policies/prp-scripts/translate_policy.py > human_policy.pol
cd $PROJECT_DIR