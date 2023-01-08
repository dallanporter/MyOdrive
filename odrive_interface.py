axis = {
    "error": None,
    "step_dir_active": None,
    "last_drv_fault": None,
    "steps": None,
    "current_state": None,
    "requested_state": None,
    "is_homed": None,
    "config": {
        "startup_motor_calibration": None,
        "startup_encoder_index_search": None,
        "startup_encoder_offset_calibration": None,
        "startup_closed_loop_control": None,
        "startup_homing": None,
        "enable_step_dir": None,
        "step_dir_always_on": None,
        "enable_sensorless_mode": None,
        "watchdog_timeout": None,
        "enable_watchdog": None,
        "step_gpio_pin": None,
        "dir_gpio_pin": None,
        "calibration_lockin": {
            "current": None,
            "ramp_time": None,
            "ramp_distance": None,
            "accel": None,
            "vel": None
        },
        "sensorless_ramp": None,
        "general_lockin": None,
        "can": None
            
    },
    "motor": None,
    "controller": None,
    "encoder": None,
    "acim_estimator": None,
    "sensorless_estimator": None,
    "trap_traj": None,
    "min_endstop": None,
    "max_endstop": None,
    "mechanical_brake": None,
    "task_times": {       
        "thermistor_update": None,
            "encoder_update": None,
            "sensorless_estimator_update": None,
            "endstop_update": None,
            "can_heartbeat": None,
            "controller_update": None,
            "open_loop_controller_update": None,
            "acim_estimator_update": None,
            "motor_update": None,
            "current_controller_update": None,
            "dc_calib": None,
            "current_sense": None,
            "pwm_update": None
        } 
}

odrive = {
    "error":None,
    "vbus_voltage":None,
    "ibus":None,
    "ibus_report_filter_k":None,
    "serial_number":None,
    "hw_version_major":None,
    "hw_version_minor":None,
    "hw_version_variant":None,
    "fw_version_major":None,
    "fw_version_minor":None,
    "fw_version_revision":None,
    "fw_version_unreleased":None,
    "brake_resistor_armed":None,
    "brake_resistor_saturated":None,
    "brake_resistor_current":None,
    "n_evt_sampling":None,
    "n_evt_control_loop":None,
    "task_timers_armed":None,
    "system_stats": {
        "uptime": None,
        "min_heap_space": None,
        "max_stack_usage_axis": None,
        "max_stack_usage_usb": None,
        "max_stack_usage_uart": None,
        "max_stack_usage_can": None,
        "max_stack_usage_startup": None,
        "max_stack_usage_analog": None,
        "stack_size_axis": None,
        "stack_size_usb": None,
        "stack_size_uart": None,
        "stack_size_startup": None,
        "stack_size_can": None,
        "stack_size_analog": None,
        "prio_axis": None,
        "prio_usb": None,
        "prio_uart": None,
        "prio_startup": None,
        "prio_can": None,
        "prio_analog": None,
        "usb": {
            "rx_cnt": None,
            "tx_cnt": None,
            "tx_overrun_cnt": None
        },
        "i2c": {
            "addr": None,
            "addr_match_cnt": None,
            "rx_cnt": None,
            "error_cnt": None
        },
    },
    "user_config_loaded": None,
    "misconfigured": None,
    "oscilloscope": None,
    "can": None,
    "test_property": None,
    "otp_valid": None,

    "axis0": {}
        
}