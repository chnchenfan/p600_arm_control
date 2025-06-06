# !/bin/bash
gnome-terminal --window -e "bash -c 'roscore; exec bash'" \
--tab -e "bash -c 'rosrun plotjuggler plotjuggler; exec bash'" \
