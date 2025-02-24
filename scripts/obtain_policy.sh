#! /bin/bash

PROJECT_DIR=$(pwd)

./planner-for-relevant-policies/src/prp \
"$PROJECT_DIR/config/PDDL/ltlf_domain.pddl" \
"$PROJECT_DIR/config/PDDL/ltlf_problem.pddl" \
--dump-policy 2

mv elapsed.time "$PROJECT_DIR/config/PDDL/elapsed.time"
mv output "$PROJECT_DIR/config/PDDL/output"
mv output.sas "$PROJECT_DIR/config/PDDL/output.sas"
mv plan_numbers_and_cost "$PROJECT_DIR/config/PDDL/plan_numbers_and_cost"
mv policy.fsap "$PROJECT_DIR/config/PDDL/policy.fsap"
mv policy.out "$PROJECT_DIR/config/PDDL/policy.out"
mv sas_plan "$PROJECT_DIR/config/PDDL/sas_plan"