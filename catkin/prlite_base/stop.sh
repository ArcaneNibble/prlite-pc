# reset pid gains many times to make sure the message gets through
# note that message is sent with both dstaddr = 2 and dstaddr = 4
rostopic pub /wheel_pid i2c_net_packets/wheel_pid_gains -1 -- '{p0: 0, i0: 0, d0: 0, p1: 0, i1: 0, d1: 0, rev0: true, rev1: false, dstaddr: 2}' &
rostopic pub /wheel_pid i2c_net_packets/wheel_pid_gains -1 -- '{p0: 0, i0: 0, d0: 0, p1: 0, i1: 0, d1: 0, rev0: true, rev1: false, dstaddr: 4}' &
rostopic pub /wheel_pid i2c_net_packets/wheel_pid_gains -1 -- '{p0: 0, i0: 0, d0: 0, p1: 0, i1: 0, d1: 0, rev0: true, rev1: false, dstaddr: 2}' &
rostopic pub /wheel_pid i2c_net_packets/wheel_pid_gains -1 -- '{p0: 0, i0: 0, d0: 0, p1: 0, i1: 0, d1: 0, rev0: true, rev1: false, dstaddr: 4}' &
rostopic pub /wheel_pid i2c_net_packets/wheel_pid_gains -1 -- '{p0: 0, i0: 0, d0: 0, p1: 0, i1: 0, d1: 0, rev0: true, rev1: false, dstaddr: 2}' &
rostopic pub /wheel_pid i2c_net_packets/wheel_pid_gains -1 -- '{p0: 0, i0: 0, d0: 0, p1: 0, i1: 0, d1: 0, rev0: true, rev1: false, dstaddr: 4}'
