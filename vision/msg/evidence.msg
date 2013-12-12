#### Time stamp of the msg ####
time stamp

#### Group number (0-9)####
uint8 group_number

#### Image which shows detected object ####
sensor_msgs/Image image_evidence

#### String for indicating which object was recognized in the image, if any ####
string object_id

# object_id should be one of the following
# ''  : empty string, this means no object was recognized
# 'tomato'
# 'lion'
# 'giraffe'
# 'lemon'
# 'banana'
# 'pepper'
# 'watermellon'
# 'tiger'
#  etc....

