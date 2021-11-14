#!/usr/bin/env /lib/init/init-d-script
### BEGIN INIT INFO
# Provides:          inm_router
# Required-Start:    $syslog $time $remote_fs $named $network
# Required-Stop:     $syslog $time $remote_fs $named $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: route INM messages
# Description:       Debian init script to start the Inter-Node Messaging
#                    router daemon.
### END INIT INFO

NAME="inm_router"
COMMAND_NAME="inm_router.py"
#DAEMON="/usr/local/bin/start_inm_router.sh"
DAEMON="/usr/local/bin/$COMMAND_NAME"

# These are arguments for the daemon process.
DAEMON_ARGS="@/usr/local/etc/inm/router_args"

# init-d-script will do this for us unless we specify PIDFILE=none
#PIDFILE="/var/run/$NAME.pid"

PYTHONPATH="/usr/local/lib/inm/"

do_start_prepare() {
	# These are arguments for start-stop-daemon.
	START_ARGS="$START_ARGS --background --make-pidfile"
	# Export PYTHONPATH so we can find the INM module.
	export PYTHONPATH
}

do_stop_cmd_override() {
	start-stop-daemon --stop --quiet --oknodo --retry=TERM/30/KILL/5 \
	    $STOP_ARGS \
	    ${PIDFILE:+--pidfile ${PIDFILE}} --name ${COMMAND_NAME}
	RETVAL="$?"
	[ "$RETVAL" = 2 ] && return 2
	# Many daemons don't delete their pidfiles when they exit.
	rm -f $PIDFILE
	return $RETVAL
}
