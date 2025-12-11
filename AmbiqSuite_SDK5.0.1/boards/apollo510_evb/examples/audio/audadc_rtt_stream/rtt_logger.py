#
# Helper script for jlinkrttlogger
#
import subprocess
import time

timestr = time.strftime("%Y%m%d-%H-%M-%S")

#file name for pcm raw data capture
bin_name = timestr + ".pcm"

#run rtt logger. Please change device name and RTTAddress accordingly.
subprocess.run(["jlinkrttlogger.exe", "-Device", 'AP510NFA-CBR', "-If", "SWD", "-speed", "4000", "-RTTAddress", "0x20087484", "-RTTChannel", "1", bin_name])
