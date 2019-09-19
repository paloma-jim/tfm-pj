# Trabajo de Fin de Máster - Inteligencia Artificial


## Models for traveling salesman with drone

###Files
* 20140810T123437v1: contains a test for 10 clients. 
* mainFSTSP.py: contains model FSTSP' with TSP solver based on the algorithm Path Cheapest Arc(PCA)
* mainFSTSP_GLS.py: contains model FSTSP' with TSP solver based on Guided Local Search (GLS)
* mainPDSTSP.py: contains model PDSTSP' with TSP solver based on PCA and PMS based on: Genetic Algorithm (GA), Shortest Job First(SJF) and random
* mainPDSTSP_GLS.py: contains model PDSTSP' with TSP solver based on GLS and PMS based on: Genetic Algorithm (GA), Shortest Job First(SJF) and random
* run_examples.sh: contains a script to execute the test for 10 clients with the 4 models defined above
* ga.py: contains the genetic algorithm for the PMS
* FSTSP_Result: contains the results after evaluating our model FSTSP' with the results from Murray&Chu
* PDSTSP_Result: contains the results after evaluating our model PDSTSP' with the results from Murray&Chu

###Execution of the script: `sh run_examples.sh`

###Execution of other problem in mainFSTSP.py or mainFSTSP_GLS.py
* Indicate the location of the file with the problem in `BASE_PATH = "20140810T123437v1/"`
* Indicate the number of clients to serve in `num_clients`
* Indicate the maximum time of flight for the drone in `drone_battery_lifetime`
* Run the file: `python3 mainFSTSP.py` or `python3 mainFSTSP_GLS.py`

###Execution of other problem in mainPDSTSP.py or mainPDSTSP_GLS.py
* Indicate the location of the file with the problem in `BASE_PATH = "20140810T123437v1/"`
* Indicate the number of clients to serve in `num_clients`
* Indicate the number of drones available `num_drones`
* Indicate the maximum time of flight for the drone in `max_drone_time`
* Run the file: `python3 mainPDSTSP.py` or `python3 mainPDSTSP_GLS.py`
