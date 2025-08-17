#!/bin/bash

pwd=`pwd`

package_name="mrs_uav_status"
binary="MrsUavStatus_Status"
node_name="uav_status"

this_pkg_path=$(ros2 pkg prefix $package_name)/share/$package_name

config_public=$this_pkg_path/config/public/default.yaml
platform_config=""
custom_config=""
colorscheme="COLORSCHEME_DARK"

[ -z $UAV_NAME ] && uav_name=uav1 || uav_name=$UAV_NAME
[ -z $USE_SIM_TIME ] && use_sim_time=false || use_sim_time=$USE_SIM_TIME

if [ ! -z "$PROFILES" ]; then

  is_light=$(echo "$PROFILES" | grep "COLORSCHEME_LIGHT" | wc -l)

  if [[ "$is_light" == "1" ]]; then
    colorscheme="COLORSCHEME_LIGHT"
  fi
fi

params=(
  "pwd"             "string" "$pwd"
  "colorscheme"     "string" "$colorscheme"
  "enable_profiler" "bool"   "false"
  "use_sim_time"    "bool"   "$use_sim_time"
  "start_minimized" "bool"   "false"
  'config_public'   "string" "$config_public"
  'platform_config' "string" "$platform_config"
  'custom_config'   "string" "$custom_config"
)

remaps=(
  "$node_name/uav_status_in"            "uav_status_acquisition/uav_status"
  "$node_name/uav_status_short_in"      "uav_status_acquisition/uav_status_short"
  "$node_name/gimbal_command_out"       "tarot_gimbal/gimbal_command"
  "$node_name/reference_out"            "control_manager/reference"
  "$node_name/trajectory_reference_out" "control_manager/trajectory_reference"
  "$node_name/set_constraints_out"      "constraint_manager/set_constraints"
  "$node_name/set_estimator_out"        "estimation_manager/change_estimator"
  "$node_name/set_gains_out"            "gain_manager/set_gains"
  "$node_name/set_controller_out"       "control_manager/switch_controller"
  "$node_name/set_tracker_out"          "control_manager/switch_tracker"
  "$node_name/hover_out"                "control_manager/hover"
  "$node_name/profiler"                 "profiler"
)

## --------------------------------------------------------------
## |                     the automatic part                     |
## --------------------------------------------------------------

CMD="ros2 run $package_name $binary __ns:=/$uav_name __node:=$node_name --ros-args"

for ((i=0; i < ${#params[*]}; i++));
do
  ((i%3==0)) && PARAM_NAME[$i/3]="${params[$i]}"
  ((i%3==1)) && PARAM_TYPE[$i/3]="${params[$i]}"
  ((i%3==2)) && PARAM_VALUE[$i/3]="${params[$i]}"
done

for ((i=0; i < ${#PARAM_NAME[*]}; i++)); do

  if [ "${PARAM_TYPE[$i]}" == "string" ]; then

    CMD="$CMD -p ${PARAM_NAME[$i]}:=\'${PARAM_VALUE[$i]}\'"

  elif [ "${PARAM_TYPE[$i]}" == "bool" ]; then

    CMD="$CMD -p ${PARAM_NAME[$i]}:=${PARAM_VALUE[$i]}"

  else

    CMD="$CMD -p ${PARAM_NAME[$i]}:=${PARAM_VALUE[$i]}"

  fi

done

for ((i=0; i < ${#remaps[*]}; i++));
do
  ((i%2==0)) && REMAP_FROM[$i/2]="${remaps[$i]}"
  ((i%2==1)) && REMAP_TO[$i/2]="${remaps[$i]}"
done

for ((i=0; i < ${#REMAP_FROM[*]}; i++)); do

  CMD="$CMD -r ${REMAP_FROM[$i]}:=${REMAP_TO[$i]}"

done

eval "$CMD"
