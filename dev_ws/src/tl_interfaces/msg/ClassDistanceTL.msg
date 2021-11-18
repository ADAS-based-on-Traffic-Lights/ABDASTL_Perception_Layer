# This message contains:
# CLASSES which is an array of integers, that goes from 1 to 4, each corresponding to a TL class (off, green yellow,$
# DISTANCES of all the TLs detected. They are sorted from the closest one to the fartest one. 
std_msgs/Header header
int8[] classes
float32[] distances

