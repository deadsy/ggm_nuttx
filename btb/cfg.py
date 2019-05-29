#------------------------------------------------------------------------------
"""

BtB configuration

"""
#------------------------------------------------------------------------------

# base path
base_path = '/home/jasonh/work/ggm'

# build path
build_path = base_path + '/build'

# download archive path
archive_path = base_path + '/dl'

# build status path
status_path = base_path + '/status'

# path to cross compilation tools
xtools = '/opt/gcc-arm-none-eabi-8-2018-q4-major/bin/arm-none-eabi-'

# git hash of nuttx source
nuttx_hash = 'HEAD'

# git hash of apps source
apps_hash = 'HEAD'

# debug flag
debug = False
#debug = True

# number of log lines added to the bad build file
log_lines = 20

#------------------------------------------------------------------------------
