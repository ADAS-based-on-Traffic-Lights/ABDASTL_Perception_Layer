# This message contains: 
# CLASSES which is an array of integers, that goes from 1 to 4, each corresponding to a TL class (off, green yellow, and red)
# BOUNDINGBOXES which is an array of integers, it consist of ymin, xmin, ymax, xmax from the bounding boxes detected (if multiple bbs then the other coordinates are concatenated in the array). 
std_msgs/Header header
int8[] classes 
int16[] boundingboxes
