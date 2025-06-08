import subprocess
import datetime
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

# Get Git commit
try:
    commit = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).decode().strip()
except:
    commit = "unknown"

# Get Git branch
try:
    branch = subprocess.check_output(['git', 'branch', '--show-current']).decode().strip()
except:
    branch = "unknown"

# Get date/time
now = datetime.datetime.now()
build_date = now.strftime("%Y-%m-%d")
build_time = now.strftime("%H:%M:%S")

# Append definitions
env.Append(CPPDEFINES=[
    ('GIT_COMMIT_ID', '\\"{}\\"'.format(commit)),
    ('GIT_CURRENT_BRANCH', '\\"{}\\"'.format(branch)),
    ('BUILD_DATE', '\\"{}\\"'.format(build_date)),
    ('BUILD_TIME', '\\"{}\\"'.format(build_time))
])

