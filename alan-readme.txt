Make sure roscore is running.
roscd prlite

roscore &
./pc_485net_raw/src/pc_485net_raw.py &
./net_485net_packet_handler/src/485net_packet_sorter.py &
#ONLY WORKS IN /prlite directory
./net_485net_id_handler/src/id_server.py &
#ONLY WORKS IN /prlite directory
./prlite_base/bin/base_controller &

wait 20 seconds

For testing
./prlite_base/send_vel.sh 0 0 5
./prlite_base/send_vel.sh 0 0 0


Fix firmware (ONLY ONE DEVICE ON CHAIN)
./net_485net_firmware_utils/src/485net_firmware_tool.py 0e
where 0e is the address
Run it until it works (up to 2 times)

