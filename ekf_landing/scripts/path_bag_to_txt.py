import os, sys
import time
import rosbag

if (len(sys.argv) == 1):
    listOfBagFiles = [f for f in os.listdir(".") if f[-4:] == ".bag"]	#get list of only bag files in current dir.
    numberOfFiles = str(len(listOfBagFiles))
    print "reading " + numberOfFiles + " bagfiles in current directory: \n"
    for f in listOfBagFiles:
        print f
    print "\n press ctrl+c in the next 3 seconds to cancel \n"
    time.sleep(1)
else:
    print ("argument is not supported yet, usage: python path_bag_to_txt.py --> automatically find bag files in the same folder and generate csv files")
    sys.exit(1)


os.makedirs("csv_files")
count = 0
for bagFile in listOfBagFiles:
    count += 1
    print "reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile
    #access bag
    bag = rosbag.Bag(bagFile)
    bagContents = bag.read_messages()
    bagName = bag.filename

    #get list of topics from the bag
    listOfTopics = []
    for topic, msg, t in bagContents:
        if topic not in listOfTopics:
            listOfTopics.append(topic)

    for topicName in listOfTopics:
        tmp_counter = 0
        for msg in bag.read_messages(topicName):
            tmp_counter += 1

        topic_name = topicName.split('/')
        if len(topic_name)>2:
            f=open("csv_files/" + bagName[:-4] + "_" + topic_name[1] + "_" + topic_name[2] + '.csv', "a")
        else:
            f=open("csv_files/" + bagName[:-4] + "_" + topic_name[1] + '.csv', "a")

        msg_counter = 0
        for subtopic, msg, t in bag.read_messages(topicName):
            if msg_counter == tmp_counter-1:
                for i in range(len(msg.poses)):
                    if i==0:
                        offset=msg.poses[i].header.stamp
                    tmp_time = float((msg.poses[i].header.stamp-offset).secs) + float((msg.poses[i].header.stamp-offset).nsecs)/1000000000.0
                    f.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n"%(tmp_time , msg.poses[i].pose.position.x, msg.poses[i].pose.position.y, msg.poses[i].pose.position.z, msg.poses[i].pose.orientation.x, msg.poses[i].pose.orientation.z, msg.poses[i].pose.orientation.y, msg.poses[i].pose.orientation.w))
            msg_counter += 1
        f.close()
    bag.close()
print "Done reading all " + numberOfFiles + " bag files."
