import traci
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import time
import os
import sys

# ------------------ SUMO Setup ------------------
if 'SUMO_HOME' not in os.environ:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)

sumoBinary = "sumo-gui"  # use "sumo" for command line
sumoCmd = [sumoBinary, "-c", "myConfig.sumocfg"]

# ------------------ Fuzzy Logic Setup ------------------
queue = ctrl.Antecedent(np.arange(0, 51, 1), 'queue')
urgency = ctrl.Antecedent(np.arange(0, 1.1, 0.1), 'urgency')
green_time = ctrl.Consequent(np.arange(0, 61, 1), 'green_time')

queue['low'] = fuzz.trimf(queue.universe, [0, 0, 15])
queue['medium'] = fuzz.trimf(queue.universe, [10, 25, 40])
queue['high'] = fuzz.trimf(queue.universe, [30, 50, 50])

urgency['low'] = fuzz.trimf(urgency.universe, [0, 0, 0.4])
urgency['medium'] = fuzz.trimf(urgency.universe, [0.2, 0.5, 0.8])
urgency['high'] = fuzz.trimf(urgency.universe, [0.6, 1.0, 1.0])

green_time['short'] = fuzz.trimf(green_time.universe, [0, 10, 20])
green_time['medium'] = fuzz.trimf(green_time.universe, [15, 25, 35])
green_time['long'] = fuzz.trimf(green_time.universe, [30, 45, 55])
green_time['very_long'] = fuzz.trimf(green_time.universe, [50, 60, 60])

rule1 = ctrl.Rule(queue['low'] & urgency['low'], green_time['short'])
rule2 = ctrl.Rule(queue['low'] & urgency['medium'], green_time['medium'])
rule3 = ctrl.Rule(queue['low'] & urgency['high'], green_time['long'])
rule4 = ctrl.Rule(queue['medium'] & urgency['low'], green_time['medium'])
rule5 = ctrl.Rule(queue['medium'] & urgency['medium'], green_time['long'])
rule6 = ctrl.Rule(queue['medium'] & urgency['high'], green_time['very_long'])
rule7 = ctrl.Rule(queue['high'] & urgency['low'], green_time['long'])
rule8 = ctrl.Rule(queue['high'] & urgency['medium'], green_time['very_long'])
rule9 = ctrl.Rule(queue['high'] & urgency['high'], green_time['very_long'])

system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])
fuzzy_ctrl = ctrl.ControlSystemSimulation(system)

# ------------------ Helper Functions ------------------

def get_fuzzy_green(queue_len, urgency_level):
    fuzzy_ctrl.input['queue'] = queue_len
    fuzzy_ctrl.input['urgency'] = urgency_level
    fuzzy_ctrl.compute()
    return fuzzy_ctrl.output['green_time']


def perform_preemption(tl_id, ev_lane, queue_len, urgency):
    """Safely preempts signal for emergency vehicle"""
    print(f"\n🚨 Emergency detected on {ev_lane} at light {tl_id}")

    # Step 1: compute fuzzy green
    green_duration = get_fuzzy_green(queue_len, urgency)
    print(f"→ Recommended emergency green time: {green_duration:.1f} s")

    # Step 2: safely transition to emergency green
    current_phase = traci.trafficlight.getPhase(tl_id)
    print(f"Current phase: {current_phase} (switching...)")
    traci.trafficlight.setPhaseDuration(tl_id, 3)  # yellow
    traci.simulationStep()
    time.sleep(2)
    traci.trafficlight.setPhase(tl_id, current_phase + 1)  # EV green
    traci.trafficlight.setPhaseDuration(tl_id, green_duration)

    # Step 3: simulate until EV clears
    ev_present = True
    while ev_present:
        traci.simulationStep()
        evs = [vid for vid in traci.vehicle.getIDList() if "emergency" in vid]
        if not evs:
            ev_present = False
    print("✅ Emergency cleared, restoring normal signal sequence.")
    traci.trafficlight.setPhase(tl_id, (current_phase + 2) % traci.trafficlight.getPhaseNumber(tl_id))


# ------------------ Main SUMO Control Loop ------------------

def run():
    traci.start(sumoCmd)
    print("\n✅ Connected to SUMO simulation\n")

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # Step 1: detect emergency vehicles
        for veh_id in traci.vehicle.getIDList():
            veh_type = traci.vehicle.getTypeID(veh_id)
            if "emergency" in veh_type:
                lane_id = traci.vehicle.getLaneID(veh_id)
                tl_id = traci.trafficlight.getIDList()[0]  # assume one intersection

                # Step 2: check if this lane currently has red light
                state = traci.trafficlight.getRedYellowGreenState(tl_id)
                lane_index = traci.lane.getIndex(lane_id)
                if state[lane_index] == 'r':
                    queue_len = traci.lane.getLastStepVehicleNumber(lane_id)
                    perform_preemption(tl_id, lane_id, queue_len, urgency=1.0)

        time.sleep(0.1)

    traci.close()
    print("\nSUMO simulation ended.")


if __name__ == "__main__":
    run()
