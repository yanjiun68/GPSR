#!/usr/bin/env bash

SERVICE="/rx200/get_motor_registers"
TYPE="interbotix_xs_msgs/srv/RegisterValues"
JOINT="elbow"

read_reg()
{
    local REG="$1"

    ros2 service call "$SERVICE" "$TYPE" \
        "{cmd_type: 'single', name: '$JOINT', reg: '$REG', value: 0}" \
        2>/dev/null |
        grep -oP '(?<=values=\[)[-0-9]+' |
        head -n1
}

decode_hw_error()
{
    local e="$1"

    if [[ -z "$e" ]]; then
        echo "READ_ERROR"
        return
    fi

    if (( e == 0 )); then
        echo "OK"
        return
    fi

    local result=""

    (( e & 1  )) && result+="INPUT_VOLTAGE "
    (( e & 4  )) && result+="OVERHEATING "
    (( e & 8  )) && result+="ENCODER "
    (( e & 16 )) && result+="ELECTRICAL_SHOCK "
    (( e & 32 )) && result+="OVERLOAD "

    echo "$result"
}

decode_mode()
{
    case "$1" in
        0)  echo "Current" ;;
        1)  echo "Velocity" ;;
        3)  echo "Position" ;;
        4)  echo "Extended Position" ;;
        5)  echo "Current-based Position" ;;
        16) echo "PWM" ;;
        *)  echo "Unknown ($1)" ;;
    esac
}

while true; do

    TEMP=$(read_reg Present_Temperature)
    VOLT_RAW=$(read_reg Present_Input_Voltage)
    CURRENT_RAW=$(read_reg Present_Current)
    PWM_RAW=$(read_reg Present_PWM)
    ERROR=$(read_reg Hardware_Error_Status)
    MODE=$(read_reg Operating_Mode)

    if [[ -n "$VOLT_RAW" ]]; then
        VOLT=$(awk "BEGIN {printf \"%.1f\", $VOLT_RAW * 0.1}")
    else
        VOLT="?"
    fi

    if [[ -n "$CURRENT_RAW" ]]; then
        CURRENT=$(awk "BEGIN {printf \"%.0f\", $CURRENT_RAW * 2.69}")
    else
        CURRENT="?"
    fi

    clear

    echo "======================================"
    echo " RX200 ELBOW DYNAMIXEL MONITOR"
    echo " Joint: elbow (ID 4)"
    echo "======================================"
    echo
    echo "Temperature : ${TEMP:-?} °C"
    echo "Voltage     : $VOLT V"
    echo "Current     : $CURRENT mA"
    echo "Current raw : ${CURRENT_RAW:-?}"
    echo "PWM raw     : ${PWM_RAW:-?}"
    echo
    echo "HW Error    : ${ERROR:-?}"
    echo "Status      : $(decode_hw_error "${ERROR:-0}")"
    echo
    echo "Mode        : $(decode_mode "$MODE")"
    echo "Mode raw    : ${MODE:-?}"
    echo
    echo "Ctrl+C to exit"

    sleep 1
done
