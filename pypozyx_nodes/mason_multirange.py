#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from pypozyx import *
from sensor_msgs.msg import Range
from serial.tools.list_ports import comports
from pypozyx_nodes.msg import uwb, uwb_array
import timeit

import sys
import signal
def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def pozyx_setup():

	rospy.init_node('pypozyx_ranging_node')

	serial_ports = []
	anchor_ids = []
	ranging_protocol = 'fast'

	# Get serial ports
	tag_id = rospy.get_param('~tag_id')
	serial_ports = rospy.get_param('~serial_ports').split(';')
	#for port in comports():
	#	if is_pozyx_port(port):
	#		serial_ports.append(port.device)
	
	# Get anchors
	anchor_names = rospy.get_param('~anchor_ids').split(';')

	for i in range(len(anchor_names)):
		anchor_ids.append(int(anchor_names[i], 16))

	# Get protocol
	_ranging_protocol = rospy.get_param('~ranging_protocol')

	if _ranging_protocol == 'fast':
		ranging_protocol = POZYX_RANGE_PROTOCOL_FAST
	else:
		ranging_protocol = POZYX_RANGE_PROTOCOL_PRECISION

	return tag_id, serial_ports, anchor_ids, ranging_protocol, anchor_names


def ranging_execute(pozyx_list, range_publisher, anchor_ids, ranging_protocol, range_history, failure_counter, anchor_ids_mask, tag_id):

    pub_data_array = uwb_array()
    for i in range(len(pozyx_list)):
        pozyx = pozyx_list[i]
        device_range = DeviceRange()
        pozyx.setRangingProtocol(ranging_protocol, None)

        for j in range(len(anchor_ids)):
            range_msg = Range()
            range_msg.header.stamp = rospy.Time.now()
            pub_data = uwb()
            pub_data.tag_id = tag_id
            pub_data.anchor_id = anchor_names[j]
            past_range = range_history[i][j]
            status = pozyx.doRanging(anchor_ids[j], device_range, None)
            device_range.distance /= 1000.0
            range_msg.field_of_view = device_range.RSS
			# Send the previous range for following occasions:
			# 1) Status has failed
			# 2) Device range is 0.0 upon success
			# 3) Device range is larger than 10*the previous range upon success
	
            if status == POZYX_SUCCESS and device_range.distance != 0.0:
                #if device_range.distance == 0.0:
                #    range_msg.range = past_range
                if device_range.distance >= 10*past_range and past_range > 0 and device_range.distance>0.5:
                    range_msg.range = past_range
                else:
                    range_msg.range = device_range.distance
                    range_history[i][j] = device_range.distance
                failure_counter[i][j] = 0
                anchor_ids_mask[j] = True

                pub_data.range = range_msg
				#range_pub_list[i][j].publish(range_msg)

			# If FAILURE, send the previous range
			# If FAILURE persists too long, regard it as 'connection lost'
            else:
                if anchor_ids_mask[j] == True:
                    failure_counter[i][j] += 1
                    if failure_counter[i][j] >= 10:
                        range_msg.range = 0.0
                        anchor_ids_mask[j] = False	
                    else:
                        range_msg.range = past_range
                        pub_data.range = range_msg
                        #range_pub_list[i][j].publish(range_msg)

            pub_data_array.uwb_array.append(pub_data)
        range_publisher.publish(pub_data_array)
    return range_history, failure_counter, anchor_ids_mask


if __name__ == "__main__":

    tag_ids = ['6a2a', '6a04', '6a5a', '6a56'] #TODO

    (tag_id, serial_ports, anchor_ids, ranging_protocol, anchor_names) = pozyx_setup()

    num_tags = len(serial_ports)
    num_anchors = len(anchor_ids)

    anchor_ids_mask = [False]*num_anchors

    counter = 0

    range_history = [[0.0]*num_anchors for j in range(num_tags)]
    failure_counter = [[0]*num_anchors for j in range(num_tags)]
    pozyx_serial = []

    range_pub = rospy.Publisher("range", uwb_array, queue_size=100)
    # Setup parameters
    for i in range(num_tags):
        for j in range(num_anchors): 
            anchor_ids_mask[j] = True
        _pozyx = PozyxSerial(serial_ports[i])
        pozyx_serial.append(_pozyx)

    # doRanging
    while True:
        try:
            range_history, failure_counter, anchor_ids_mask = ranging_execute(pozyx_serial, range_pub, anchor_ids, ranging_protocol, range_history, failure_counter, anchor_ids_mask, tag_id)
        #print(anchor_ids_mask)
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)


