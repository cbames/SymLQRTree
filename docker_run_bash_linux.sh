#! /bin/bash
echo "Drake Docker container for LINUX"
if [ "$#" != "1" ]; then 
	echo "Please supply one argument: a relative path to a directory to mount."
	exit 1
else
	docker run -it --network=host --user=$(id -u) \
	                --env="DISPLAY" \
	                --volume="/etc/group:/etc/group:ro" \
	                --volume="/etc/shadow:/etc/shadow:ro" \
	                --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
	                --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	                --volume="/etc/passwd:/etc/passwd:ro" \
	                --rm \
			        -v "$(pwd)/$2":/rss_code drake \
			        /bin/bash -c "cd /rss_code && /bin/bash"
fi
