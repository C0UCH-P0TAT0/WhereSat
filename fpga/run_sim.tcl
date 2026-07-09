# WhereSat Automated Simulation Script

# 1. Open the Vivado Project
open_project C:\Users\a\Desktop\WhereSat\fpga\build\WhereSat\WhereSat.xpr

# 2. Reset the simulation directory to ensure a clean state
# (Clears out old snapshots and log files)
reset_simulation -simset sim_1 -mode behavioral

# 3. Launch the behavioral simulation
launch_simulation -simset sim_1 -mode behavioral

# 4. Run the simulation
# Since the Testbench contains $finish, this will run until the 
# processing_done logic triggers the end of the frame.
run all

# 5. Close the simulation engine
close_sim

# 6. Exit Vivado
exit