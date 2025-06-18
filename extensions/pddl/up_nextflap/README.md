# up-nextflap
# NEXTFLAP Unified Planning integrator 
This is the NEXTFLAP UP integrator. NEXTFLAP is an expressive temporal and numeric planner supporting planning problems involving Boolean and numeric state variables, instantaneous and durative actions. A distinctive feature of NEXTFLAP is the ability to reason over problems with a prevalent numeric structure, which may involve linear and non-linear numeric conditions and effects. NEXTFLAP favors the capability of modeling expressive problems; indeed it handles negative and disjunctive preconditions as well as existential and universal expressions. NEXTFLAP can be used as a satisficing planner and as a partial-order plan validator. NEXTFLAP is written completely in C++.
NEXTFLAP is a numeric extension of [TFLAP planner](https://grps.webs.upv.es/downloadPaper.php?paperId=238), and uses the Z3 Theorem Prover to check the numeric constraints and ensure consistency of plans.

## Installation
After cloning this repository run ```pip install up-nextflap```. up-nextflap can also be installed through the unified-planning framework with the command ```pip install unified-planning[nextflap]```.

If there is no version available for your platform, a ```setup.py``` script (```nextflap``` folder) is also provided. It can assist you in the build process.

## Planning approaches of UP supported
Classical, Numeric and Temporal Planning

Partial-Order Plan Validator

## Operative modes of UP currently supported
One shot planning
Plan Validator

## Acknowledgments

<img src="https://www.aiplan4eu-project.eu/wp-content/uploads/2021/07/euflag.png" width="60" height="40">

This library is being developed for the AIPlan4EU H2020 project (https://aiplan4eu-project.eu) that is funded by the European Commission under grant agreement number 101016442.
