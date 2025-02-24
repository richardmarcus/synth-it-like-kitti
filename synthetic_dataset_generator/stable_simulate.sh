#!/bin/bash
# Function to start both scripts

CARLA_Path="$1"
Python_Path="$2"
#paths should be like:
#".../carla/Dist/CARLA_Shipping_0.9.15-1-g614d4338c/LinuxNoEditor/"
#".../synthetic_dataset_generator/"

a_script="$CARLA_Path/CarlaUE4.sh -RenderOffScreen"
b_script="$Python_Path/carla_save_cam_lidar.py --save-data"

start_scripts() {


    # Start a.sh in the background 
    $a_script 2>errorUE.log &
    a_pid=$!
    echo "SIM_INFO: Started UE4.sh with PID $a_pid"

    # Wait for 30 seconds before starting b.py
    sleep 45

    # Start b.py in the background and capture its output (stderr) for error checking
    python $b_script >debug.log 2>error.log &
    b_pid=$!
    echo "SIM_INFO: Started carla_save_cam_lidar.py .py with PID $b_pid"
}

# Function to kill both scripts
kill_scripts() {
    echo "Killing UE4.sh (PID $a_pid) and carla_save_cam_lidar.py (PID $b_pid) (using pkill -f and correct process names)..."
    kill $a_pid
    #kill $b_pid
    pkill -f "$a_script"
    pkill -f "CarlaUE4-Linux-Shipping"
    pkill -f "$b_script"
    wait $a_pid 2>/dev/null
    wait $b_pid 2>/dev/null

    #wait 5 seconds
    sleep 5
}

kill_python_script_only() {
    echo "Killing carla_save_cam_lidar.py (PID $b_pid)..."
    kill $b_pid
    pkill -f "$b_script"
    wait $b_pid 2>/dev/null
}

start_python_script_only() {
    python $b_script 2>error.log &
    b_pid=$!
    echo "SIM_INFO: Started carla_save_cam_lidar.py with PID $b_pid"
}


log_file="error.log"
UE4_log_file="errorUE.log"
# Function to check if the log file has been updated in the last minute
check_log_activity() {
    if [ -e "$log_file" ]; then
        last_modification=$(stat -c %Y "$log_file")
        current_time=$(date +%s)
        time_diff=$((current_time - last_modification))
        echo "SIM_INFO: Last modification of $log_file was $time_diff seconds ago."
        if [ $time_diff -gt 60 ]; then
            echo "SIM_INFO: No activity in $log_file for over 1 minute. Restarting python script..."
            return 1  # Indicate that a restart is needed
        fi
    else
        echo "SIM_INFO: Log file $log_file does not exist. Skipping check."
    fi
    return 0  # No restart needed
}

# Trap Ctrl+C (SIGINT) and run cleanup
trap ctrl_c INT

function ctrl_c() {
    echo "SIM_INFO: Ctrl+C pressed. Exiting..."
    kill_scripts  # Clean up and kill processes
    exit 0  # Exit the script
}


#sleep 1 hour

# Start the scripts initially
start_scripts


max_runtime=216000

start_time=$(date +%s)
# Monitor for errors or inactivity

while true; do
    # Calculate the elapsed time
    current_time=$(date +%s)
    elapsed_time=$((current_time - start_time))

    # Exit the loop if the maximum runtime is reached
    echo "SIM_INFO: Elapsed time: $elapsed_time seconds (Max runtime: $max_runtime seconds)"
    if [ $elapsed_time -ge $max_runtime ]; then
        echo "SIM_INFO: Max runtime of $max_runtime seconds reached. Exiting..."
        kill_scripts
        exit 0
    fi

    # Check for Python script errors by looking for "Traceback" in error.log or Segmentation fault
    if grep -q "Traceback" $log_file || grep -q "Segmentation fault" $UE4_log_file || grep -q  "ERROR"  $UE4_log_file || grep -q  "Aborted"  $UE4_log_file || grep -q "caught" $UE4_log_file || grep -q "dumped" $UE4_log_file  ; then
        echo "SIM_INFO: Error detected in CARLAUE4, restarting both scripts..."
        
        # Kill both scripts if error is detected in carla_save_cam_lidar.py
        kill_scripts
        
        # Clear error log
        > $log_file
        > $UE4_log_file
        
        # Restart both scripts
        start_scripts
    fi

    echo "SIM_INFO: Checking for log activity..."
    # Check if the log file has been updated in the last minute
    if ! check_log_activity; then
        # If no changes in the monitored log, restart both scripts
        kill_python_script_only
        start_python_script_only
    fi

    # Sleep for 5 seconds before checking again
    sleep 10
done