# command line parameters are id, target
rostopic pub /linear_actuator_target i2c_net_packets/linact_target -1 -- '{dstaddr: 6, which: '$1', min: '$(($2-13))', max: '$(($2+13))'}'
