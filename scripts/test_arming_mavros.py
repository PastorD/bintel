import mavros
from mavros import command

mavros.set_namespace()
command.arming(True)
