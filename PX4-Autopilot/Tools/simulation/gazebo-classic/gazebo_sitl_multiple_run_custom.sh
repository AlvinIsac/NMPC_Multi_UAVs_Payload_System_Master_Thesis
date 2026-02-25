#!/bin/bash
# run multiple instances of the 'px4' binary, with the gazebo SITL simulation
# It assumes px4 is already built, with 'make px4_sitl_default gazebo'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example gazebo can be run like this:
#./Tools/gazebo_sitl_multiple_run.sh -n 10 -m iris

function cleanup() {
	pkill -x px4
	pkill gzclient
	pkill gzserver
}

function spawn_model() {
	MODEL=$1
	N=$2 #Instance Number
	X=$3
	Y=$4
	X=${X:=0.0}
	Y=${Y:=0.0} #Y=${Y:=$((2*${N}))}
	Z=${Z:=0.0}
	R=${R:=0.0}
	P=${P:=0.0}
	Yaw=${Yaw:=0.0}
	K=${K:=0.0}



	SUPPORTED_MODELS=("iris", "x500" "plane", "standard_vtol", "typhoon_h480", "payload", "takeoff_plat","goal_line","payload_line","camera_target")
	if [[ " ${SUPPORTED_MODELS[*]} " != *"$MODEL"* ]];
	then
		echo "ERROR: Currently only vehicle model $MODEL is not supported!"
		echo "       Supported Models: [${SUPPORTED_MODELS[@]}]"
		trap "cleanup" SIGINT SIGTERM EXIT
		exit 1
	fi

# Assigning to each model the initial pose
	if [[ "$MODEL" == "takeoff_plat" ]];
	then
		if [[ $N -eq 0 ]];
		then
			X=-1.855
			Y=0.0
			Z=0.085
			Yaw=0.0
		elif [[ $N -eq 1 ]];
		then
			X=0.93
			Y=-1.598
			Z=0.085
			Yaw=2.09333
		elif [[ $N -eq 2 ]];
		then
			X=0.93
			Y=1.598
			Z=0.085
			Yaw=4.18667
		fi
	elif ([ "$MODEL" == "iris" ] || [ "$MODEL" == "x500" ]);
	then
		K=$((N - 4))
		if [[ $N -eq 4 ]];
		then
			X=-1.695
			Y=0.0
			Z=0.21
			Yaw=0.0
			# X=-0.262
			# Y=-1.487
			# Z=0.0
			# Yaw=1.395 #80 deg
		elif [[ $N -eq 5 ]];
		then
			X=0.85
			Y=-1.4684
			Z=0.21
			Yaw=2.09333
			# X=-0.262
			# Y=1.487
			# Z=0.0
			# Yaw=-1.395
		elif [[ $N -eq 6 ]];
		then
			X=0.85
			Y=1.4684
			Z=0.21
			Yaw=4.18667

		fi
	elif [[ "$MODEL" == "payload" ]];
	then
          if [[ $N -eq 3 ]];
          then
          	X=0.0
           # X=0.075
           Y=0.0
					 Z=0.0
           Yaw=0.0
				 	fi
	elif [[ "$MODEL" == "goal_line" ]];
	then

					# X=0.0
					X=497.4
					Y=-0.3
					# Y=497.0
					Z=7.005
					P=1.570796325
				  # Yaw=0.7853981625
					Yaw=0.0
	elif [[ "$MODEL" == "payload_line" ]];
	then

					# X=0.0
					X=497.4
					Y=-0.3
					# Y=497.0
					Z=5.11
					P=1.570796325
					# Yaw=0.7853981625
					Yaw=0.0
	elif [[ "$MODEL" == "camera_target" ]];
	then
					# X=-1.707106782
					# Y=-1.307106782
					# X=1.0
					# Y=-4.0
					X=-3.0
					Y=-1.0
					Z=5.8050
	fi



	working_dir="$build_path/rootfs/$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $N in $(pwd)"

	if ([ "$MODEL" == "iris" ] || [ "$MODEL" == "x500" ]);
	then

		export PX4_UXRCE_DDS_NS=${MODEL}_$(($K + 1))

		$build_path/bin/px4 -i $K -d "$build_path/etc" -w sitl_${MODEL}_${N} -s etc/init.d-posix/rcS >out.log 2>err.log &

		set --
		set -- ${@} ${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py
		set -- ${@} ${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/${MODEL}/${MODEL}.sdf.jinja
		set -- ${@} ${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic
		set -- ${@} --mavlink_tcp_port $((4560+${K}))
		set -- ${@} --mavlink_udp_port $((14560+${K}))
		set -- ${@} --mavlink_id $((1+${K}))
		set -- ${@} --gst_udp_port $((5600+${K}))
		set -- ${@} --video_uri $((5600+${K}))
		set -- ${@} --mavlink_cam_udp_port $((14530+${K}))
		set -- ${@} --qgc_udp_port $((14550+${K}))
		set -- ${@} --sdk_udp_port $((14540+${K}))
		set -- ${@} --output-file /tmp/${MODEL}_$(($K + 1)).sdf

		python3 ${@}

		echo "Spawning ${MODEL}_$(($K + 1)) at ${X} ${Y} ${Z}"

	gz model --spawn-file=/tmp/${MODEL}_$(($K + 1)).sdf --model-name=${MODEL}_$(($K + 1)) -x ${X} -y ${Y} -z ${Z} -R ${R} -P ${P} -Y ${Yaw}


	else

		$build_path/bin/px4 -i $N -d "$build_path/etc" -w sitl_${MODEL}_${N} -s etc/init.d-posix/rcS >out.log 2>err.log &

		set --
		set -- ${@} ${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py
		set -- ${@} ${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/${MODEL}/${MODEL}.sdf.jinja
		set -- ${@} ${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic
		set -- ${@} --output-file /tmp/${MODEL}_${N}.sdf

		python3 ${@}

		echo "Spawning ${MODEL}_${N} at ${X} ${Y} ${Z}"

	gz model --spawn-file=/tmp/${MODEL}_${N}.sdf --model-name=${MODEL}_${N} -x ${X} -y ${Y} -z ${Z} -R ${R} -P ${P} -Y ${Yaw}

	fi


	popd &>/dev/null

}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Usage: $0 [-n <num_vehicles>] [-m <vehicle_model>] [-w <world>] [-s <script>]"
	echo "-s flag is used to script spawning vehicles e.g. $0 -s iris:3,plane:2"
	exit 1
fi

while getopts n:m:w:s:t:l: option
do
	case "${option}"
	in
		n) NUM_VEHICLES=${OPTARG};;
		m) VEHICLE_MODEL=${OPTARG};;
		w) WORLD=${OPTARG};;
		s) SCRIPT=${OPTARG};;
		t) TARGET=${OPTARG};;
		l) LABEL=_${OPTARG};;
	esac
done

num_vehicles=${NUM_VEHICLES:=3}
world=${WORLD:=empty}
target=${TARGET:=px4_sitl_default}
vehicle_model=${VEHICLE_MODEL:=$MODEL}
export PX4_SIM_MODEL=gazebo-classic_${vehicle_model}

echo ${SCRIPT}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../.."

build_path=${src_path}/build/${target}
mavlink_udp_port=14560
mavlink_tcp_port=4560

echo "killing running instances"
pkill -x px4 || true

sleep 1

source ${src_path}/Tools/simulation/gazebo-classic/setup_gazebo.bash ${src_path} ${src_path}/build/${target}

# To use gazebo_ros ROS2 plugins
if [[ -n "$ROS_VERSION" ]] && [ "$ROS_VERSION" == "2" ]; then
	ros_args="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
else
	ros_args=""
fi

echo "Starting gazebo"
gzserver ${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/${world}.world --verbose $ros_args &
sleep 5

n=0
if [ -z ${SCRIPT} ]; then
	if [ $num_vehicles -gt 255 ]
	then
		echo "Tried spawning $num_vehicles vehicles. The maximum number of supported vehicles is 255"
		exit 1
	fi

	while [ $n -lt $num_vehicles ]; do
		spawn_model ${vehicle_model} $n #$(($n + 1))
		n=$(($n + 1))
	done
else
	IFS=,
	for target in ${SCRIPT}; do
		target="$(echo "$target" | tr -d ' ')" #Remove spaces
		target_vehicle=$(echo $target | cut -f1 -d:)
		target_number=$(echo $target | cut -f2 -d:)
		target_x=$(echo $target | cut -f3 -d:)
		target_y=$(echo $target | cut -f4 -d:)

		if [ $n -gt 255 ]
		then
			echo "Tried spawning $n vehicles. The maximum number of supported vehicles is 255"
			exit 1
		fi

		m=0
		while [ $m -lt ${target_number} ]; do
			export PX4_SIM_MODEL=gazebo-classic_${target_vehicle}${LABEL}
			spawn_model ${target_vehicle}${LABEL} $n $target_x $target_y #$(($n + 1))
			m=$(($m + 1))
			n=$(($n + 1))
		done
	done

fi
trap "cleanup" SIGINT SIGTERM EXIT

echo "Starting gazebo client"
gzclient
