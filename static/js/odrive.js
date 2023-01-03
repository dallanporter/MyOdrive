{
    "version": "0.0.1",
    "ns": "com.odriverobotics",
    "summary": "ODrive Interface Definitions",
    "dictionary": [
        "ODrive"
    ],
    "userdata": {
        "c_preamble": "#include <tuple>\nusing float2D = std::pair<float, float>;\nstruct Iph_ABC_t { float phA; float phB; float phC; };\n"
    },
    "interfaces": {
        "ODrive": {
            "c_is_class": true,
            "brief": "Toplevel interface of your ODrive.",
            "doc": "The odrv0, odrv1, ... objects that appear in odrivetool implement this\ntoplevel interface.\n",
            "attributes": {
                "error": {
                    "nullflag": "NONE",
                    "flags": {
                        "CONTROL_ITERATION_MISSED": {
                            "brief": "At least one control iteration was missed.",
                            "doc": "The main control loop is supposed to runs at a fixed frequency.\nIf the device is computationally overloaded (e.g. too many active\ncomponents) it's possible that one or more control iterations\nare skipped.\n"
                        },
                        "DC_BUS_UNDER_VOLTAGE": {
                            "brief": "The DC voltage fell below the limit configured in `config.dc_bus_undervoltage_trip_level`.",
                            "doc": "Confirm that your power leads are connected securely. For initial\ntesting a 12V PSU which can supply a couple of amps should be\nsufficient while the use of low current \u00e2\u20ac\u02dcwall wart\u00e2\u20ac\u2122 plug packs may\nlead to inconsistent behaviour and is not recommended.\n\nYou can monitor your PSU voltage using liveplotter in odrivetool\nby entering `start_liveplotter(lambda: [odrv0.vbus_voltage])`. If\nyou see your votlage drop below `config.dc_bus_undervoltage_trip_level`\n(default: ~ 8V) then you will trip this error. Even a relatively\nsmall motor can draw multiple kW momentary and so unless you have\na very large PSU or are running of a battery you may encounter\nthis error when executing high speed movements with a high current\nlimit. To limit your PSU power draw you can limit your motor\ncurrent and/or velocity limit `Axis:controller.config.vel_limit` and\n`Axis:motor.config.current_lim`.\n"
                        },
                        "DC_BUS_OVER_VOLTAGE": {
                            "brief": "The DC voltage exceeded the limit configured in `config.dc_bus_overvoltage_trip_level`.",
                            "doc": "Confirm that you have a brake resistor of the correct value\nconnected securely and that `config.brake_resistance` is set to\nthe value of your brake resistor.\n\nYou can monitor your PSU voltage using liveplotter in odrivetool\nby entering `start_liveplotter(lambda: [odrv0.vbus_voltage])`. If\nduring a move you see the voltage rise above your PSU\u00e2\u20ac\u2122s nominal\nset voltage then you have your brake resistance set too low. This\nmay happen if you are using long wires or small gauge wires to\nconnect your brake resistor to your odrive which will added extra\nresistance. This extra resistance needs to be accounted for to\nprevent this voltage spike. If you have checked all your\nconnections you can also try increasing your brake resistance by\n~ 0.01 ohm at a time to a maximum of 0.05 greater than your brake\nresistor value.\n"
                        },
                        "DC_BUS_OVER_REGEN_CURRENT": {
                            "doc": "Current flowing back into the power supply exceeded `config.dc_max_negative_current`.\nThis can happen if your brake resistor is disabled or unable to handle the braking current.\n\nCheck that `config.enable_brake_resistor` is `True` and that\n`(V_power_supply / Brake_resistance) > (total motor.config.current_lim + total motor.config.current_lim_margin)`.\n"
                        },
                        "DC_BUS_OVER_CURRENT": {
                            "doc": "Too much current was pulled from the power supply. `ibus` exceeded `config.dc_max_positive_current`.\n"
                        },
                        "BRAKE_DEADTIME_VIOLATION": null,
                        "BRAKE_DUTY_CYCLE_NAN": null,
                        "INVALID_BRAKE_RESISTANCE": {
                            "doc": "`config.brake_resistance` is non-positive or NaN. Make sure that `config.brake_resistance` is a positive number."
                        }
                    }
                },
                "vbus_voltage": {
                    "type": "readonly float32",
                    "unit": "V",
                    "brief": "Voltage on the DC bus as measured by the ODrive."
                },
                "ibus": {
                    "type": "readonly float32",
                    "unit": "A",
                    "brief": "Current on the DC bus as calculated by the ODrive.",
                    "doc": "A positive value means that the ODrive is consuming power from the power supply,\na negative value means that the ODrive is sourcing power to the power supply.\n\nThis value is equal to the sum of the motor currents and the brake resistor currents.\nThe motor currents are measured, the brake resistor current is calculated based on\n`config.brake_resistance`.\n"
                },
                "ibus_report_filter_k": {
                    "type": "float32",
                    "doc": "Filter gain for the reported `ibus`. Set to a value below 1.0 to get a smoother\nline when plotting `ibus`. Set to 1.0 to disable. This filter is only applied to\nthe reported value and not for internal calculations.\n"
                },
                "serial_number": "readonly uint64",
                "hw_version_major": "readonly uint8",
                "hw_version_minor": "readonly uint8",
                "hw_version_variant": "readonly uint8",
                "fw_version_major": "readonly uint8",
                "fw_version_minor": "readonly uint8",
                "fw_version_revision": "readonly uint8",
                "fw_version_unreleased": {
                    "type": "readonly uint8",
                    "doc": "0 for official releases, 1 otherwise"
                },
                "brake_resistor_armed": "readonly bool",
                "brake_resistor_saturated": "readonly bool",
                "brake_resistor_current": {
                    "type": "readonly float32",
                    "doc": "Commanded brake resistor current"
                },
                "n_evt_sampling": {
                    "type": "readonly uint32",
                    "doc": "Number of input sampling events since startup (modulo 2^32)"
                },
                "n_evt_control_loop": {
                    "type": "readonly uint32",
                    "doc": "Number of control loop iterations since startup (modulo 2^32)"
                },
                "task_timers_armed": {
                    "type": "bool",
                    "doc": "Set by a profiling application to trigger sampling of a single\ncontrol iteration. Cleared by the device as soon as the sampling\nis complete.\n"
                },
                "task_times": {
                    "c_is_class": false,
                    "attributes": {
                        "sampling": "TaskTimer",
                        "control_loop_misc": "TaskTimer",
                        "control_loop_checks": "TaskTimer",
                        "dc_calib_wait": "TaskTimer"
                    }
                },
                "system_stats": {
                    "c_is_class": false,
                    "attributes": {
                        "uptime": "readonly uint32",
                        "min_heap_space": "readonly uint32",
                        "max_stack_usage_axis": "readonly uint32",
                        "max_stack_usage_usb": "readonly uint32",
                        "max_stack_usage_uart": "readonly uint32",
                        "max_stack_usage_can": "readonly uint32",
                        "max_stack_usage_startup": "readonly uint32",
                        "max_stack_usage_analog": "readonly uint32",
                        "stack_size_axis": "readonly uint32",
                        "stack_size_usb": "readonly uint32",
                        "stack_size_uart": "readonly uint32",
                        "stack_size_startup": "readonly uint32",
                        "stack_size_can": "readonly uint32",
                        "stack_size_analog": "readonly uint32",
                        "prio_axis": "readonly int32",
                        "prio_usb": "readonly int32",
                        "prio_uart": "readonly int32",
                        "prio_startup": "readonly int32",
                        "prio_can": "readonly int32",
                        "prio_analog": "readonly int32",
                        "usb": {
                            "c_is_class": false,
                            "attributes": {
                                "rx_cnt": "readonly uint32",
                                "tx_cnt": "readonly uint32",
                                "tx_overrun_cnt": "readonly uint32"
                            }
                        },
                        "i2c": {
                            "c_is_class": false,
                            "attributes": {
                                "addr": "readonly uint8",
                                "addr_match_cnt": "readonly uint32",
                                "rx_cnt": "readonly uint32",
                                "error_cnt": "readonly uint32"
                            }
                        }
                    }
                },
                "user_config_loaded": "readonly uint32",
                "misconfigured": {
                    "type": "readonly bool",
                    "doc": "If this property is true, something is bad in the configuration. The\nODrive can still be used in this state but the user should investigate\nwhich setting is problematic. This variable does not cover all\nmisconfigurations.\n\nPossible causes:\n - A GPIO was set to a mode that it doesn't support\n - A GPIO was set to a mode for which the corresponding feature was\n   not enabled. Example: `GPIO_MODE_UART_A` was used without enabling\n   `config.enable_uart_a`.\n - A feature was enabled which is not supported on this hardware.\n   Example: `config.enable_uart_c` set to true on ODrive v3.x.\n - A GPIO was used as an interrupt input for two internal components\n   or two GPIOs that are mutually exclusive in their interrupt\n   capability were both used as interrupt input.\n   Example: `Axis:config.step_gpio_pin` of both axes were set to the same GPIO.\n  \n"
                },
                "oscilloscope": {
                    "type": "Oscilloscope"
                },
                "can": {
                    "type": "Can"
                },
                "test_property": "uint32",
                "otp_valid": "readonly bool"
            },
            "functions": {
                "test_function": {
                    "in": {
                        "delta": "int32"
                    },
                    "out": {
                        "cnt": "int32"
                    }
                },
                "get_adc_voltage": {
                    "in": {
                        "gpio": "uint32"
                    },
                    "out": {
                        "voltage": "float32"
                    },
                    "doc": "Reads the ADC voltage of the specified GPIO. The GPIO should be in `GPIO_MODE_ANALOG_IN`.}"
                },
                "save_configuration": {
                    "out": {
                        "success": "bool"
                    },
                    "doc": "Saves the current configuration to non-volatile memory and reboots the board."
                },
                "erase_configuration": {
                    "doc": "Resets all `config` variables to their default values and reboots the controller"
                },
                "reboot": {
                    "doc": "Reboots the controller without saving the current configuraiton"
                },
                "enter_dfu_mode": {
                    "doc": "Enters the Device Firmware Update mode"
                },
                "get_interrupt_status": {
                    "in": {
                        "irqn": {
                            "type": "int32",
                            "doc": "-12...-1: processor interrupts, 0...239: NVIC interrupts"
                        }
                    },
                    "out": {
                        "status": {
                            "type": "uint32",
                            "doc": "bit 31:     enabled (1) or disabled (0)\nbits 30:8:  number of times the interrupt fired (modulo 0x800000)\nbits 7:0:   priority (0 is highest priority)\n0xffffffff if the specified number is not a valid interrupt number.\n"
                        }
                    },
                    "doc": "Returns information about the specified interrupt number."
                },
                "get_dma_status": {
                    "in": {
                        "stream_num": {
                            "type": "uint8",
                            "doc": "0...7: DMA1 streams, 8...15: DMA2 streams"
                        }
                    },
                    "out": {
                        "status": {
                            "type": "uint32",
                            "doc": "bit 31:     zero if the stream's configuration is equal to the reset state\nbits 4:2:   channel\nbits 1:0:   priority (3 is highest priority)\n0xffffffff if the specified number is not a valid DMA stream number.\n"
                        }
                    },
                    "doc": "Returns information about the specified DMA stream."
                },
                "get_gpio_states": {
                    "out": {
                        "status": {
                            "type": "uint32"
                        }
                    },
                    "doc": "Returns the logic states of all GPIOs. Bit i represents the state of GPIOi."
                },
                "get_drv_fault": {
                    "out": {
                        "drv_fault": "uint64"
                    }
                },
                "clear_errors": {
                    "doc": "Clear all the errors of this device including all contained submodules."
                }
            }
        },
        "ODrive.Config": {
            "c_is_class": false,
            "attributes": {
                "enable_uart_a": {
                    "type": "bool",
                    "brief": "Enables/disables UART_A.",
                    "doc": "You also need to set the corresponding GPIOs to GPIO_MODE_UART_A.\nRefer to [interfaces](interfaces.md) to see which pins support UART_A.\nChanging this requires a reboot.\n"
                },
                "enable_uart_b": {
                    "type": "bool",
                    "brief": "Enables/disables UART_B.",
                    "doc": "You also need to set the corresponding GPIOs to GPIO_MODE_UART_B.\nRefer to [interfaces](interfaces.md) to see which pins support UART_B.\nChanging this requires a reboot.\n"
                },
                "enable_uart_c": {
                    "type": "bool",
                    "doc": "Not supported on ODrive v3.x."
                },
                "uart_a_baudrate": {
                    "type": "uint32",
                    "unit": "baud/s",
                    "brief": "Defines the baudrate used on the UART interface.",
                    "doc": "Some baudrates will have a small timing error due to hardware limitations.\n\nHere's an (incomplete) list of baudrates for ODrive v3.x:\n\n+-------------+---------------+-----------+\n| Configured  | Actual        | Error [%] |\n+=============+===============+===========+\n| 1.2 KBps    | 1.2 KBps      | 0         |\n+-------------+---------------+-----------+\n| 2.4 KBps    | 2.4 KBps      | 0         |\n+-------------+---------------+-----------+\n| 9.6 KBps    | 9.6 KBps      | 0         |\n+-------------+---------------+-----------+\n| 19.2 KBps   | 19.195 KBps   | 0.02      |\n+-------------+---------------+-----------+\n| 38.4 KBps   | 38.391 KBps   | 0.02      |\n+-------------+---------------+-----------+\n| 57.6 KBps   | 57.613 KBps   | 0.02      |\n+-------------+---------------+-----------+\n| 115.2 KBps  | 115.068 KBps  | 0.11      |\n+-------------+---------------+-----------+\n| 230.4 KBps  | 230.769 KBps  | 0.16      |\n+-------------+---------------+-----------+\n| 460.8 KBps  | 461.538 KBps  | 0.16      |\n+-------------+---------------+-----------+\n| 921.6 KBps  | 913.043 KBps  | 0.93      |\n+-------------+---------------+-----------+\n| 1.792 MBps  | 1.826 MBps    | 1.9       |\n+-------------+---------------+-----------+\n| 1.8432 MBps | 1.826 MBps    | 0.93      |\n+-------------+---------------+-----------+\n\n\nFor more information refer to Section 30.3.4 and Table 142 (the column with f_PCLK = 42 MHz) in the\n`STM datasheet <https://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf>`__.\n"
                },
                "uart_b_baudrate": {
                    "type": "uint32",
                    "unit": "baud/s",
                    "brief": "Defines the baudrate used on the UART interface.",
                    "doc": "See `uart_a_baudrate` for details."
                },
                "uart_c_baudrate": {
                    "type": "uint32",
                    "doc": "Not supported on ODrive v3.x."
                },
                "enable_can_a": {
                    "type": "bool",
                    "doc": "Enables CAN. Changing this setting requires a reboot.\n"
                },
                "enable_i2c_a": {
                    "type": "bool",
                    "doc": "Enables I2C. The I2C pins on ODrive v3.x are in conflict with CAN.\nThis setting has no effect if `enable_can_a` is also true.\nThis setting has no effect on ODrive v3.2 or earlier.\nChanging this setting requires a reboot.\n"
                },
                "usb_cdc_protocol": {
                    "type": "StreamProtocolType",
                    "doc": "The protocol that's being run on the device's virtual COM port on\nUSB.\nNote that the ODrive has two independent interfaces on USB: One\nis the virtual COM port (affected by this option) and the other\none is a vendor specific interface which always runs Fibre.\nSo changing this option does not affect the working of odrivetool.\n"
                },
                "uart0_protocol": "StreamProtocolType",
                "uart1_protocol": "StreamProtocolType",
                "uart2_protocol": "StreamProtocolType",
                "max_regen_current": {
                    "type": "float32",
                    "unit": "Amps",
                    "doc": "The bus current allowed to flow back to the power supply before the brake resistor module will start shunting current.\n"
                },
                "brake_resistance": {
                    "type": "float32",
                    "unit": "ohm",
                    "brief": "Value of the brake resistor connected to the ODrive.",
                    "doc": "If you set this to a lower value than the true brake resistance\nthen the ODrive will not meed the `max_regen_current` constraint\nduring braking, that is it will sink more than `max_regen_current`\ninto the power supply. Some power supplies don't like this.\n\nIf you set this to a higher value than the true brake resistance\nthen the ODrive will unnecessarily burn more power than required\nduring braking.\n"
                },
                "enable_brake_resistor": {
                    "type": "bool",
                    "brief": "Enable/disable the use of a brake resistor.",
                    "doc": "Setting this to False even though a brake resistor is connected is\nharmless. Setting this to True even though no brake resistor is\nconnected can break the power supply.\nChanges to this value require a reboot to take effect.\n"
                },
                "dc_bus_undervoltage_trip_level": {
                    "type": "float32",
                    "unit": "V",
                    "brief": "Minimum voltage below which the motor stops operating."
                },
                "dc_bus_overvoltage_trip_level": {
                    "type": "float32",
                    "unit": "V",
                    "brief": "Maximum voltage above which the motor stops operating.",
                    "doc": "This protects against cases in which the power supply fails to dissipate\nthe brake power if the brake resistor is disabled.\nThe default is 26V for the 24V board version and 52V for the 48V board version.\n"
                },
                "enable_dc_bus_overvoltage_ramp": {
                    "type": "bool",
                    "status": "experimental",
                    "brief": "Enables the DC bus overvoltage ramp feature.",
                    "doc": "If enabled, if the measured DC voltage exceeds `dc_bus_overvoltage_ramp_start`,\nthe ODrive will sink more power than usual into the the brake resistor\nin an attempt to bring the voltage down again.\n\nThe brake duty cycle is increased by the following amount:\n\n * `ODrive:vbus_voltage` == `dc_bus_overvoltage_ramp_start`  =>  brake_duty_cycle += 0%\n * `ODrive:vbus_voltage` == `dc_bus_overvoltage_ramp_end`  =>  brake_duty_cycle += 100%\n\nRemarks:\n - This feature is active even when all motors are disarmed.\n - This feature is disabled if `brake_resistance` is non-positive.\n"
                },
                "dc_bus_overvoltage_ramp_start": {
                    "type": "float32",
                    "status": "experimental",
                    "brief": "See `enable_dc_bus_overvoltage_ramp`.",
                    "doc": "Do not set this lower than your usual `ODrive:vbus_voltage`, unless you like fried brake resistors."
                },
                "dc_bus_overvoltage_ramp_end": {
                    "type": "float32",
                    "status": "experimental",
                    "brief": "See `enable_dc_bus_overvoltage_ramp`.",
                    "doc": "Must be larger than `dc_bus_overvoltage_ramp_start`, otherwise the ramp feature is disabled."
                },
                "dc_max_positive_current": {
                    "type": "float32",
                    "unit": "A",
                    "brief": "Max current the power supply can source."
                },
                "dc_max_negative_current": {
                    "type": "float32",
                    "unit": "A",
                    "brief": "Max current the power supply can sink.",
                    "doc": "You most likely want a non-positive value here. Set to -INFINITY to disable.\nNote: This should be greater in magnitude than `max_regen_current`\n"
                },
                "error_gpio_pin": {
                    "type": "uint32"
                },
                "gpio3_analog_mapping": {
                    "type": "Endpoint",
                    "c_name": "analog_mappings[3]",
                    "doc": "Make sure the corresponding GPIO is in `GPIO_MODE_ANALOG_IN`."
                },
                "gpio4_analog_mapping": {
                    "type": "Endpoint",
                    "c_name": "analog_mappings[4]",
                    "doc": "Make sure the corresponding GPIO is in `GPIO_MODE_ANALOG_IN`."
                }
            }
        },
        "ODrive.Can": {
            "c_is_class": true,
            "attributes": {
                "error": {
                    "nullflag": "NONE",
                    "flags": {
                        "DUPLICATE_CAN_IDS": null
                    }
                },
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "baud_rate": {
                            "type": "uint32",
                            "c_setter": "set_baud_rate"
                        },
                        "protocol": {
                            "type": "Protocol"
                        }
                    }
                }
            }
        },
        "ODrive.Endpoint": {
            "c_is_class": false,
            "attributes": {
                "endpoint": "endpoint_ref",
                "min": "float32",
                "max": "float32"
            }
        },
        "ODrive.Axis": {
            "c_is_class": true,
            "attributes": {
                "error": {
                    "nullflag": "NONE",
                    "flags": {
                        "INVALID_STATE": {
                            "brief": "An invalid state was requested.",
                            "doc": "You tried to run a state before you are allowed to. Typically you\ntried to run encoder calibration or closed loop control before the\nmotor was calibrated, or you tried to run closed loop control\nbefore the encoder was calibrated.\n"
                        },
                        "MOTOR_FAILED": {
                            "bit": 6,
                            "doc": "Check `motor.error` for more information."
                        },
                        "SENSORLESS_ESTIMATOR_FAILED": null,
                        "ENCODER_FAILED": {
                            "doc": "Check `encoder.error` for more information."
                        },
                        "CONTROLLER_FAILED": null,
                        "WATCHDOG_TIMER_EXPIRED": {
                            "bit": 11,
                            "brief": "The axis watchdog timer expired.",
                            "doc": "An amount of time greater than `config.watchdog_timeout` passed\nwithout the watchdog being fed.\n"
                        },
                        "MIN_ENDSTOP_PRESSED": {
                            "brief": "The min endstop was pressed"
                        },
                        "MAX_ENDSTOP_PRESSED": {
                            "brief": "The max endstop was pressed"
                        },
                        "ESTOP_REQUESTED": {
                            "brief": "The estop message was sent over CAN"
                        },
                        "HOMING_WITHOUT_ENDSTOP": {
                            "bit": 17,
                            "doc": "the min endstop was not enabled during homing"
                        },
                        "OVER_TEMP": {
                            "doc": "Check `motor.error` for more details."
                        },
                        "UNKNOWN_POSITION": {
                            "doc": "There isn't a valid position estimate available."
                        }
                    }
                },
                "step_dir_active": "readonly bool",
                "last_drv_fault": "readonly uint32",
                "steps": {
                    "type": "readonly int64",
                    "brief": "The current commanded position, in steps, while in `step_dir` mode"
                },
                "current_state": {
                    "type": "readonly AxisState",
                    "brief": "The current state of the axis"
                },
                "requested_state": {
                    "type": "AxisState",
                    "brief": "The user's commanded axis state",
                    "doc": "This is used to command the axis to change state or perform certain routines.\nValues input here will be \"consumed\" and queued by the state machine handler.  Thus, reading this value back will usually show `UNDEFINED` (0).\n"
                },
                "is_homed": {
                    "type": "bool",
                    "c_name": "homing_.is_homed",
                    "brief": "Whether or not the axis has been successfully homed."
                },
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "startup_motor_calibration": {
                            "type": "bool",
                            "doc": "Run motor calibration at startup, skip otherwise"
                        },
                        "startup_encoder_index_search": {
                            "type": "bool",
                            "doc": "Run encoder index search after startup, skip otherwise this only has an effect if encoder.config.use_index is also true"
                        },
                        "startup_encoder_offset_calibration": {
                            "type": "bool",
                            "doc": "Run encoder offset calibration after startup, skip otherwise"
                        },
                        "startup_closed_loop_control": {
                            "type": "bool",
                            "doc": "Enable closed loop control after calibration/startup"
                        },
                        "startup_homing": {
                            "type": "bool",
                            "doc": "Enable homing after calibration/startup"
                        },
                        "enable_step_dir": {
                            "type": "bool",
                            "doc": "Enable step/dir input after calibration.\nMake sure to set the corresponding GPIO's mode to `GPIO_MODE_DIGITAL`.\n"
                        },
                        "step_dir_always_on": {
                            "type": "bool",
                            "doc": "Keep step/dir enabled while the motor is disabled.\nThis is ignored if enable_step_dir is false.\nThis setting only takes effect on a state transition\ninto idle or out of closed loop control.\n"
                        },
                        "enable_sensorless_mode": "bool",
                        "watchdog_timeout": {
                            "type": "float32",
                            "unit": "s"
                        },
                        "enable_watchdog": "bool",
                        "step_gpio_pin": {
                            "type": "uint16",
                            "c_setter": "set_step_gpio_pin"
                        },
                        "dir_gpio_pin": {
                            "type": "uint16",
                            "c_setter": "set_dir_gpio_pin"
                        },
                        "calibration_lockin": {
                            "c_is_class": false,
                            "attributes": {
                                "current": "float32",
                                "ramp_time": "float32",
                                "ramp_distance": "float32",
                                "accel": "float32",
                                "vel": "float32"
                            }
                        },
                        "sensorless_ramp": "LockinConfig",
                        "general_lockin": "LockinConfig",
                        "can": "CanConfig"
                    }
                },
                "motor": "Motor",
                "controller": "Controller",
                "encoder": "Encoder",
                "acim_estimator": "AcimEstimator",
                "sensorless_estimator": "SensorlessEstimator",
                "trap_traj": "TrapezoidalTrajectory",
                "min_endstop": "Endstop",
                "max_endstop": "Endstop",
                "mechanical_brake": "MechanicalBrake",
                "task_times": {
                    "c_is_class": false,
                    "attributes": {
                        "thermistor_update": "TaskTimer",
                        "encoder_update": "TaskTimer",
                        "sensorless_estimator_update": "TaskTimer",
                        "endstop_update": "TaskTimer",
                        "can_heartbeat": "TaskTimer",
                        "controller_update": "TaskTimer",
                        "open_loop_controller_update": "TaskTimer",
                        "acim_estimator_update": "TaskTimer",
                        "motor_update": "TaskTimer",
                        "current_controller_update": "TaskTimer",
                        "dc_calib": "TaskTimer",
                        "current_sense": "TaskTimer",
                        "pwm_update": "TaskTimer"
                    }
                }
            },
            "functions": {
                "watchdog_feed": {
                    "doc": "Feed the watchdog to prevent watchdog timeouts."
                }
            }
        },
        "ODrive.Axis.LockinConfig": {
            "c_is_class": false,
            "attributes": {
                "current": {
                    "type": "float32",
                    "unit": "A"
                },
                "ramp_time": {
                    "type": "float32",
                    "unit": "s"
                },
                "ramp_distance": {
                    "type": "float32",
                    "unit": "rad"
                },
                "accel": {
                    "type": "float32",
                    "unit": "rad/s^2"
                },
                "vel": {
                    "type": "float32",
                    "unit": "rad/s"
                },
                "finish_distance": {
                    "type": "float32",
                    "unit": "rad"
                },
                "finish_on_vel": "bool",
                "finish_on_distance": "bool",
                "finish_on_enc_idx": "bool"
            }
        },
        "ODrive.Axis.CanConfig": {
            "c_is_class": false,
            "attributes": {
                "node_id": "uint32",
                "is_extended": "bool",
                "heartbeat_rate_ms": "uint32",
                "encoder_rate_ms": "uint32"
            }
        },
        "ODrive.ThermistorCurrentLimiter": {
            "c_is_class": false
        },
        "ODrive.OnboardThermistorCurrentLimiter": {
            "c_is_class": true,
            "attributes": {
                "temperature": {
                    "type": "readonly float32",
                    "unit": "\u00c2\u00b0C"
                },
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "temp_limit_lower": {
                            "type": "float32",
                            "doc": "The lower limit when the controller starts limiting current."
                        },
                        "temp_limit_upper": {
                            "type": "float32",
                            "doc": "The upper limit when current limit reaches 0 Amps and an over temperature error is triggered."
                        },
                        "enabled": {
                            "type": "bool",
                            "doc": "Whether this thermistor is enabled."
                        }
                    }
                }
            }
        },
        "ODrive.OffboardThermistorCurrentLimiter": {
            "c_is_class": true,
            "attributes": {
                "temperature": {
                    "type": "readonly float32",
                    "unit": "\u00c2\u00b0C"
                },
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "gpio_pin": {
                            "type": "uint16",
                            "c_setter": "set_gpio_pin"
                        },
                        "poly_coefficient_0": {
                            "type": "float32",
                            "c_name": "thermistor_poly_coeffs[0]"
                        },
                        "poly_coefficient_1": {
                            "type": "float32",
                            "c_name": "thermistor_poly_coeffs[1]"
                        },
                        "poly_coefficient_2": {
                            "type": "float32",
                            "c_name": "thermistor_poly_coeffs[2]"
                        },
                        "poly_coefficient_3": {
                            "type": "float32",
                            "c_name": "thermistor_poly_coeffs[3]"
                        },
                        "temp_limit_lower": {
                            "type": "float32",
                            "doc": "The lower limit when the controller starts limiting current."
                        },
                        "temp_limit_upper": {
                            "type": "float32",
                            "doc": "The upper limit when current limit reaches 0 Amps and an over temperature error is triggered."
                        },
                        "enabled": {
                            "type": "bool",
                            "doc": "Whether this thermistor is enabled."
                        }
                    }
                }
            }
        },
        "ODrive.Motor": {
            "c_is_class": true,
            "attributes": {
                "last_error_time": "float32",
                "error": {
                    "nullflag": "NONE",
                    "flags": {
                        "PHASE_RESISTANCE_OUT_OF_RANGE": {
                            "brief": "The measured motor phase resistance is outside of the plausible range.",
                            "doc": "During calibration the motor resistance and\n[inductance](https://en.wikipedia.org/wiki/Inductance) is measured.\nIf the measured motor resistance or inductance falls outside a set\nrange this error will be returned. Check that all motor leads are\nconnected securely.\n\nThe measured values can be viewed using odrivetool as is shown below:\n```\nIn [2]: odrv0.axis0.motor.config.phase_inductance\nOut[2]: 1.408751450071577e-05\n\nIn [3]: odrv0.axis0.motor.config.phase_resistance\nOut[3]: 0.029788672924041748\n```\nSome motors will have a considerably different phase resistance\nand inductance than this. For example, gimbal motors, some small\nmotors (e.g. < 10A peak current). If you think this applies to you\ntry increasing `config.resistance_calib_max_voltage` from\nits default value of 1 using odrivetool and repeat the motor\ncalibration process. If your motor has a small peak current draw\n(e.g. < 20A) you can also try decreasing\n`config.calibration_current` from its default value of 10A.\n\nIn general, you need\n```text\nresistance_calib_max_voltage > calibration_current * phase_resistance\nresistance_calib_max_voltage < 0.5 * vbus_voltage\n```\n"
                        },
                        "PHASE_INDUCTANCE_OUT_OF_RANGE": {
                            "brief": "The measured motor phase inductance is outside of the plausible range.",
                            "doc": "See `PHASE_RESISTANCE_OUT_OF_RANGE` for details.\n"
                        },
                        "DRV_FAULT": {
                            "bit": 3,
                            "brief": "The gate driver chip reported an error.",
                            "doc": "The ODrive v3.4 is known to have a hardware issue whereby the\nmotors would stop operating when applying high currents to M0. The\nreported error of both motors in this case is `ERROR_DRV_FAULT`.\n\nThe conjecture is that the high switching current creates large\nripples in the power supply of the DRV8301 gate driver chips, thus\ntripping its under-voltage fault detection.\n\nTo resolve this issue you can limit the M0 current to 40A. The\nlowest current at which the DRV fault was observed is 45A on one\ntest motor and 50A on another test motor. Refer to\n[this post](https://discourse.odriverobotics.com/t/drv-fault-on-odrive-v3-4/558)\nfor instructions for a hardware fix.\n"
                        },
                        "CONTROL_DEADLINE_MISSED": null,
                        "MODULATION_MAGNITUDE": {
                            "bit": 7,
                            "doc": "The bus voltage was insufficent to push the requested current\nthrough the motor.\nIf you are getting this during motor calibration, make sure that\n`config.resistance_calib_max_voltage` is no more than half\nyour bus voltage.\n\nFor gimbal motors, it is recommended to set the\n`config.calibration_current` and `config.current_lim`\nto half your bus voltage, or less.\n"
                        },
                        "CURRENT_SENSE_SATURATION": {
                            "bit": 10,
                            "doc": "The current sense circuit saturated the current sense amplifier.\nThis can be caused by setting `config.current_lim` higher than\n`config.requested_current_range`. If this happens, increase the\nrequested current range, save the configuration, and reboot the controller.\n"
                        },
                        "CURRENT_LIMIT_VIOLATION": {
                            "bit": 12,
                            "doc": "The motor current exceeded `motor.config.current_lim + motor.config.current_lim_margin`.\nThe current controller is a PI controller, so it can experience overshoot. The PI gains\nare automatically calculated based on `config.current_control_bandwidth` and the\nmotor resistance and inductance (pole placement). Some overshoot is normal, so a sensible\nsolution is to increase the current limit margin if your current limit is large.\n"
                        },
                        "MODULATION_IS_NAN": {
                            "bit": 16
                        },
                        "MOTOR_THERMISTOR_OVER_TEMP": {
                            "doc": "The motor thermistor measured a temperature above motor.motor_thermistor.config.temp_limit_upper"
                        },
                        "FET_THERMISTOR_OVER_TEMP": {
                            "doc": "The inverter thermistor measured a temperature above motor.fet_thermistor.config.temp_limit_upper"
                        },
                        "TIMER_UPDATE_MISSED": {
                            "doc": "A timer update event was missed. Perhaps the previous timer update took too much time. This is not expected in official release firmware."
                        },
                        "CURRENT_MEASUREMENT_UNAVAILABLE": {
                            "doc": "The phase current measurement is not available. The ADC failed to sample the current sensor in time. This is not expected in official release firmware."
                        },
                        "CONTROLLER_FAILED": {
                            "doc": "The motor was disarmed because the underlying controller failed. Usually this is the FOC controller."
                        },
                        "I_BUS_OUT_OF_RANGE": {
                            "doc": "The DC current sourced/sunk by this motor exceeded the configured\nhard limits. More specifically `I_bus` fell outside of the range\n`config.I_bus_hard_min` ... `config.I_bus_hard_max`.\n"
                        },
                        "BRAKE_RESISTOR_DISARMED": {
                            "doc": "An attempt was made to run the motor PWM while the brake resistor was configured as enabled\n(`config.enable_brake_resistor`) but disarmed.\n\nThe most common cause is that you just set `config.enable_brake_resistor` to `True` but didn't\narm the brake resistor yet (by either rebooting or running `odrvX.clear_errors()`).\n\nOtherwise, the brake resistor can be disarmed due to various system-wide errors.\nThe root cause will usually show up under `system` when you run `dump_errors(odrvX)`.\n\nTo re-arm the brake resistor reboot the ODrive or run `odrvX.clear_errors()`.\n"
                        },
                        "SYSTEM_LEVEL": {
                            "doc": "The motor had to be disarmed because of a system level error.\nSee `ODrive:error` for more details.\n"
                        },
                        "BAD_TIMING": {
                            "doc": "The main control loop got out of sync with the motor control loop. This could indicate that the main control loop got stuck."
                        },
                        "UNKNOWN_PHASE_ESTIMATE": {
                            "doc": "The current controller did not get a valid angle input. Maybe you didn't calibrate the encoder."
                        },
                        "UNKNOWN_PHASE_VEL": {
                            "doc": "The motor controller did not get a valid phase velocity input."
                        },
                        "UNKNOWN_TORQUE": {
                            "doc": "The motor controller did not get a valid torque input."
                        },
                        "UNKNOWN_CURRENT_COMMAND": {
                            "doc": "The current controller did not get a valid current setpoint. Maybe you didn't configure the controller correctly."
                        },
                        "UNKNOWN_CURRENT_MEASUREMENT": {
                            "doc": "The current controller did not get a valid current measurement."
                        },
                        "UNKNOWN_VBUS_VOLTAGE": {
                            "doc": "The current controller did not get a valid `ODrive:vbus_voltage` measurement."
                        },
                        "UNKNOWN_VOLTAGE_COMMAND": {
                            "doc": "The current controller did not get a valid feedforward voltage setpoint."
                        },
                        "UNKNOWN_GAINS": {
                            "doc": "The current controller gains were not configured. Run motor calibration or set `config.phase_resistance` and `config.phase_inductance` manually."
                        },
                        "CONTROLLER_INITIALIZING": {
                            "doc": "Internal value used while the controller is not yet ready to generate PWM timings."
                        },
                        "UNBALANCED_PHASES": {
                            "doc": "The motor phases are not balanced."
                        }
                    }
                },
                "is_armed": "readonly bool",
                "is_calibrated": "readonly bool",
                "current_meas_phA": {
                    "type": "readonly float32",
                    "c_getter": "current_meas_.value_or(Iph_ABC_t{0.0f, 0.0f, 0.0f}).phA"
                },
                "current_meas_phB": {
                    "type": "readonly float32",
                    "c_getter": "current_meas_.value_or(Iph_ABC_t{0.0f, 0.0f, 0.0f}).phB"
                },
                "current_meas_phC": {
                    "type": "readonly float32",
                    "c_getter": "current_meas_.value_or(Iph_ABC_t{0.0f, 0.0f, 0.0f}).phC"
                },
                "DC_calib_phA": {
                    "type": "float32",
                    "c_name": "DC_calib_.phA"
                },
                "DC_calib_phB": {
                    "type": "float32",
                    "c_name": "DC_calib_.phB"
                },
                "DC_calib_phC": {
                    "type": "float32",
                    "c_name": "DC_calib_.phC"
                },
                "I_bus": {
                    "type": "readonly float32",
                    "unit": "A",
                    "doc": "The current in the ODrive DC bus.  This is also the current seen by the power supply in most systems."
                },
                "phase_current_rev_gain": "float32",
                "effective_current_lim": {
                    "type": "readonly float32",
                    "unit": "A",
                    "doc": "This value is the internally-limited value of phase current allowed according to the set current limit and the FET and Motor thermistor limits.\n"
                },
                "max_allowed_current": {
                    "type": "readonly float32",
                    "unit": "A",
                    "doc": "Indicates the maximum current that can be measured by the current\nsensors in the current hardware configuration. This value depends on\n`config.requested_current_range`.\n"
                },
                "max_dc_calib": {
                    "type": "readonly float32",
                    "unit": "A"
                },
                "fet_thermistor": "OnboardThermistorCurrentLimiter",
                "motor_thermistor": "OffboardThermistorCurrentLimiter",
                "current_control": {
                    "c_is_class": true,
                    "attributes": {
                        "p_gain": {
                            "type": "readonly float32",
                            "c_getter": "pi_gains_.value_or(float2D{0.0f, 0.0f}).first"
                        },
                        "i_gain": {
                            "type": "readonly float32",
                            "c_getter": "pi_gains_.value_or(float2D{0.0f, 0.0f}).second"
                        },
                        "I_measured_report_filter_k": "float32",
                        "Id_setpoint": {
                            "type": "readonly float32",
                            "c_getter": "Idq_setpoint_.value_or(float2D{0.0f, 0.0f}).first"
                        },
                        "Iq_setpoint": {
                            "type": "readonly float32",
                            "c_getter": "Idq_setpoint_.value_or(float2D{0.0f, 0.0f}).second"
                        },
                        "Vd_setpoint": {
                            "type": "readonly float32",
                            "c_getter": "Vdq_setpoint_.value_or(float2D{0.0f, 0.0f}).first"
                        },
                        "Vq_setpoint": {
                            "type": "readonly float32",
                            "c_getter": "Vdq_setpoint_.value_or(float2D{0.0f, 0.0f}).second"
                        },
                        "phase": {
                            "type": "readonly float32",
                            "c_getter": "phase_.value_or(0.0f)"
                        },
                        "phase_vel": {
                            "type": "readonly float32",
                            "c_getter": "phase_vel_.value_or(0.0f)"
                        },
                        "Ialpha_measured": {
                            "type": "readonly float32",
                            "c_getter": "Ialpha_beta_measured_.value_or(float2D{0.0f, 0.0f}).first"
                        },
                        "Ibeta_measured": {
                            "type": "readonly float32",
                            "c_getter": "Ialpha_beta_measured_.value_or(float2D{0.0f, 0.0f}).second"
                        },
                        "Id_measured": {
                            "type": "readonly float32",
                            "unit": "A",
                            "doc": "The phase current measured along the \"d\" axis in the FOC control loop.  This should be close to 0 for typical surface permanent magnet motors.\n"
                        },
                        "Iq_measured": {
                            "type": "readonly float32",
                            "unit": "A",
                            "doc": "The phase current measured along the \"q\" axis in the FOC control loop.  This is the torque-generating current, so motor torque is approx. `torque_constant * Iq_measured`\n"
                        },
                        "power": {
                            "type": "readonly float32",
                            "unit": "W",
                            "doc": "The electrical power being delivered to the motor\n"
                        },
                        "v_current_control_integral_d": "float32",
                        "v_current_control_integral_q": "float32",
                        "final_v_alpha": "readonly float32",
                        "final_v_beta": "readonly float32"
                    }
                },
                "n_evt_current_measurement": {
                    "type": "readonly uint32",
                    "doc": "Number of current measurement events since startup (modulo 2^32)"
                },
                "n_evt_pwm_update": {
                    "type": "readonly uint32",
                    "doc": "Number of PWM update events since startup (modulo 2^32)"
                },
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "pre_calibrated": {
                            "type": "bool",
                            "c_setter": "set_pre_calibrated",
                            "doc": "Indicates to the ODrive that the parameters `config.phase_resistance` and `config.phase_inductance` are valid at system start.\nIf these are valid and `pre_calibrated` is set to `True`, motor calibration can be skipped.\n"
                        },
                        "pole_pairs": {
                            "type": "int32",
                            "doc": "The number of pole pairs in the motor.\nNote this is equal to 1/2 of the number of magnets (not coils!) in a typical hobby motor.\n"
                        },
                        "calibration_current": {
                            "type": "float32",
                            "doc": "The current used to measure resistance during `AXIS_STATE_MOTOR_CALIBRATION`."
                        },
                        "resistance_calib_max_voltage": {
                            "type": "float32",
                            "doc": "The maximum voltage allowed during `AXIS_STATE_MOTOR_CALIBRATION`.\nThis should be set to less than `(0.5 * vbus_voltage)`, but high enough to satisfy V=IR during motor calibration, where I is `config.calibration_current` and R is `config.phase_resistance`\n"
                        },
                        "phase_inductance": {
                            "type": "float32",
                            "unit": "henry",
                            "c_setter": "set_phase_inductance"
                        },
                        "phase_resistance": {
                            "type": "float32",
                            "unit": "ohm",
                            "c_setter": "set_phase_resistance"
                        },
                        "torque_constant": {
                            "type": "float32",
                            "unit": "N\u00c2\u00b7m/A"
                        },
                        "motor_type": "MotorType",
                        "current_lim": {
                            "type": "float32",
                            "unit": "A",
                            "doc": "Maximum commanded current allowed"
                        },
                        "current_lim_margin": {
                            "type": "float32",
                            "unit": "A",
                            "doc": "Maximum measured current allowed above `current_lim`.  Any value above `current_lim + curren_lim_margin` will throw a `CURRENT_LIMIT_VIOLATION` error."
                        },
                        "torque_lim": {
                            "type": "float32",
                            "unit": "N\u00c2\u00b7m",
                            "doc": "Maximum commanded torque allowed in the torque loop."
                        },
                        "inverter_temp_limit_lower": {
                            "type": "float32",
                            "unit": "\u00c2\u00b0C",
                            "doc": "Not implemented."
                        },
                        "inverter_temp_limit_upper": {
                            "type": "float32",
                            "unit": "\u00c2\u00b0C",
                            "doc": "Not implemented."
                        },
                        "requested_current_range": {
                            "type": "float32",
                            "unit": "A",
                            "doc": "The minimum phase current range expected to be measured.  This is used to set the current shunt amplifier gains.\nThis should be set > `current_lim + curren_lim_margin`, but as low as possible to maximize precision and accuracy of the controller.\n"
                        },
                        "current_control_bandwidth": {
                            "type": "float32",
                            "c_setter": "set_current_control_bandwidth",
                            "unit": "rad/s",
                            "doc": "Sets the PI gains of the Q and D axis FOC control according to `phase_resistance` and `phase_inductance` to create a critically-damped controller with a -3dB\nbandwidth at this frequency.\n"
                        },
                        "acim_gain_min_flux": "float32",
                        "acim_autoflux_min_Id": "float32",
                        "acim_autoflux_enable": "bool",
                        "acim_autoflux_attack_gain": "float32",
                        "acim_autoflux_decay_gain": "float32",
                        "R_wL_FF_enable": {
                            "type": "bool",
                            "doc": "Enables automatic feedforward of the R*wL term in the current controller."
                        },
                        "bEMF_FF_enable": {
                            "type": "bool",
                            "doc": "Enables automatic feedforward of the bEMF term in the current controller."
                        },
                        "I_bus_hard_min": {
                            "type": "float32",
                            "unit": "A",
                            "doc": "If the controller fails to keep this motor's DC current (`ODrive.Motor:I_bus`)\nabove this value the motor gets disarmed immediately. Most likely\nyou want a negative value here. Set to -inf to disable. Take noise\ninto account when chosing a value.\n"
                        },
                        "I_bus_hard_max": {
                            "type": "float32",
                            "unit": "A",
                            "doc": "If the controller fails to keep this motor's DC current (`ODrive.Motor:I_bus`)\nbelow this value the motor gets disarmed immediately. Usually this\nis set in conjunction with `I_bus_hard_min`. Set to inf to disable.\nTake noise into account when chosing a value.\n"
                        },
                        "I_leak_max": {
                            "type": "float32",
                            "unit": "A",
                            "doc": "In almost all scenarios, the currents on phase A, B and C should\nadd up to zero. A small amount of measurement noise is expected.\nHowever if the sum of A, B, C currents exceeds this configuration\nvalue, the motor gets disarmed immediately.\n\nNote that this feature is only works on devices with three current\nsensors (e.g. ODrive v4).\n"
                        },
                        "dc_calib_tau": "float32"
                    }
                }
            }
        },
        "ODrive.Oscilloscope": {
            "c_is_class": true,
            "attributes": {
                "size": "readonly uint32"
            },
            "functions": {
                "get_val": {
                    "in": {
                        "index": "uint32"
                    },
                    "out": {
                        "val": "float32"
                    }
                }
            }
        },
        "ODrive.AcimEstimator": {
            "c_is_class": true,
            "attributes": {
                "rotor_flux": {
                    "type": "readonly float32",
                    "unit": "A",
                    "doc": "estimated magnitude of the rotor flux"
                },
                "slip_vel": {
                    "type": "readonly float32",
                    "unit": "rad/s",
                    "doc": "estimated slip between physical and electrical angular velocity}",
                    "c_getter": "slip_vel_.any().value_or(0.0f)"
                },
                "phase_offset": {
                    "type": "readonly float32",
                    "unit": "rad",
                    "doc": "estimate offset between physical and electrical angular position}"
                },
                "stator_phase_vel": {
                    "type": "readonly float32",
                    "unit": "rad/s",
                    "doc": "calculated setpoint for the electrical velocity}",
                    "c_getter": "stator_phase_vel_.any().value_or(0.0f)"
                },
                "stator_phase": {
                    "type": "readonly float32",
                    "unit": "rad",
                    "doc": "calculated setpoint for the electrical phase}",
                    "c_getter": "stator_phase_.any().value_or(0.0f)"
                },
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "slip_velocity": "float32"
                    }
                }
            }
        },
        "ODrive.Controller": {
            "c_is_class": true,
            "attributes": {
                "error": {
                    "nullflag": "NONE",
                    "flags": {
                        "OVERSPEED": {
                            "brief": "Motor speed exceeded `config.vel_limit * config.vel_limit_tolerance` and `config.enable_overspeed_error` was enabled.",
                            "doc": "Try increasing `config.vel_limit`. The default of 2 turns per second \ngives a motor speed of only 120 RPM. Note: Even if\nyou do not command your motor to exceed `config.vel_limit`,\nsudden changes in the load placed on a motor may cause this speed\nto be temporarily exceeded, resulting in this error.\n\nYou can also try increasing `config.vel_limit_tolerance`. The\ndefault value of 1.2 means it will only allow a 20% violation of\nthe speed limit. You can set `config.enable_overspeed_error` to False\nto disable this error.\n"
                        },
                        "INVALID_INPUT_MODE": {
                            "brief": "The `config.input_mode` setting was set to an invalid value. See InputMode for available values",
                            "doc": "Input modes and control modes are separate concepts. A control mode sets the type of control to be used,\nlike position, velocity, or torque control. Input modes modify the input given (`input_pos`, etc) to\ngive desired behavior. For example, in position *control mode*, the position filter *input mode* will\nsmooth out `input_pos` commands to give smoother motion.\n"
                        },
                        "UNSTABLE_GAIN": {
                            "doc": "`motor.config.bandwidth` was set too high to create a stable controller."
                        },
                        "INVALID_MIRROR_AXIS": {
                            "doc": "An invalid `axis_to_mirror` was selected"
                        },
                        "INVALID_LOAD_ENCODER": {
                            "doc": "An invalid `load_encoder_axis` was selected"
                        },
                        "INVALID_ESTIMATE": {
                            "doc": "Indicates the encoder estimation module declined to output a linear position or velocity value and the exception was caught."
                        },
                        "INVALID_CIRCULAR_RANGE": {
                            "doc": "Indicates the encoder estimation module declined to output a circular position value and the exception was caught."
                        },
                        "SPINOUT_DETECTED": {
                            "doc": "The motor mechanical power and electrical power do not agree. This is usually\ncaused by a slipping encoder or incorrect encoder offset calibration.\nCheck that your encoder is not slipping on the motor. If using an Index pin, check\nthat you are not getting false index pulses caused by noise. This can happen if you\nare using unshielded cable for the encoder signals.\n"
                        }
                    }
                },
                "last_error_time": "float32",
                "input_pos": {
                    "type": "float32",
                    "unit": "turn",
                    "c_setter": "set_input_pos",
                    "doc": "Set the desired position of the axis.  Only valid in `CONTROL_MODE_POSITION_CONTROL`.startup.\nAlso updates the step count.\nIn `INPUT_MODE_TUNING`, this acts as a DC offset for the position sine wave.\n"
                },
                "input_vel": {
                    "type": "float32",
                    "unit": "turn/s",
                    "doc": "In `CONTROL_MODE_VELOCITY_CONTROL`, sets the desired velocity of the axis.\nIn `CONTROL_MODE_POSITION_CONTROL`, sets the feed-forward velocity of the velocity controller\nIn `INPUT_MODE_TUNING`, this acts as a DC offset for the velocity sine wave.\n"
                },
                "input_torque": {
                    "type": "float32",
                    "unit": "N\u00c2\u00b7m",
                    "doc": "In `CONTROL_MODE_TORQUE_CONTROL`, sets the desired output torque of the axis.\nIn `CONTROL_MODE_VELOCITY_CONTROL` and `CONTROL_MODE_POSITION_CONTROL`, sets the feed-forward torque of the torque controller.\nIn `INPUT_MODE_TUNING`, this acts as a DC offset for the torque sine wave.\n"
                },
                "pos_setpoint": {
                    "type": "readonly float32",
                    "unit": "turn",
                    "doc": "The position reference actually being used by the position controller.  This is the same as `input_pos` in `INPUT_MODE_PASSTHROUGH`, but may vary according to `InputMode`."
                },
                "vel_setpoint": {
                    "type": "readonly float32",
                    "unit": "turn/s",
                    "doc": "The velocity reference actually being used by the velocity controller.  This is the same as `input_vel` in `INPUT_MODE_PASSTHROUGH`, but may vary according to `InputMode`."
                },
                "torque_setpoint": {
                    "type": "readonly float32",
                    "unit": "N\u00c2\u00b7m",
                    "doc": "The torque reference actually being used by the torque controller.  This is the same as `input_vel` in `INPUT_MODE_PASSTHROUGH`, but may vary according to `InputMode`."
                },
                "trajectory_done": {
                    "type": "readonly bool",
                    "doc": "Indicates the last commanded Trapezoidal Trajectory movement is complete."
                },
                "vel_integrator_torque": {
                    "type": "float32",
                    "unit": "N\u00c2\u00b7m",
                    "doc": "The accumulated value of the velocity loop integrator"
                },
                "anticogging_valid": "bool",
                "autotuning_phase": {
                    "type": "float32",
                    "unit": "rad",
                    "doc": "The current phase angle of the `INPUT_MODE_TUNING` sine wave generator"
                },
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "gain_scheduling_width": "float32",
                        "enable_vel_limit": "bool",
                        "enable_torque_mode_vel_limit": {
                            "type": "bool",
                            "doc": "Enable velocity limit in torque control mode (requires a valid velocity estimator)."
                        },
                        "enable_gain_scheduling": {
                            "type": "bool",
                            "doc": "Enable the experimental \"gain scheduling\" module, which reduces the `pos_gain`, `vel_gain`, and `vel_integrator_gain` according to the position error.\nAlso known as \"anti-hunt\"\n"
                        },
                        "enable_overspeed_error": {
                            "type": "bool",
                            "doc": "Enables the velocity controller's overspeed error"
                        },
                        "control_mode": {
                            "type": "ControlMode",
                            "c_setter": "set_control_mode"
                        },
                        "input_mode": "InputMode",
                        "pos_gain": {
                            "type": "float32",
                            "unit": "(turn/s) / turn",
                            "doc": "units = (turn/s) / turn"
                        },
                        "vel_gain": {
                            "type": "float32",
                            "unit": "N\u00c2\u00b7m / (turn/s)",
                            "doc": "units = N\u00c2\u00b7m / (turn/s)"
                        },
                        "vel_integrator_gain": {
                            "type": "float32",
                            "unit": "N\u00c2\u00b7m / (turn/s * s)",
                            "doc": "units = N\u00c2\u00b7m / (turn/s * s)"
                        },
                        "vel_integrator_limit": {
                            "type": "float32",
                            "unit": "N\u00c2\u00b7m",
                            "doc": "Limit the integrator output (independent of proportional gain output). Set to infinity to disable. Units = N\u00c2\u00b7m"
                        },
                        "vel_limit": {
                            "type": "float32",
                            "unit": "turn/s",
                            "doc": "Infinity to disable."
                        },
                        "vel_limit_tolerance": {
                            "type": "float32",
                            "doc": "Ratio to `vel_limit`. Infinity to disable."
                        },
                        "vel_ramp_rate": {
                            "type": "float32",
                            "unit": "turn/s^2"
                        },
                        "torque_ramp_rate": {
                            "type": "float32",
                            "unit": "N\u00c2\u00b7m / sec"
                        },
                        "circular_setpoints": {
                            "type": "bool"
                        },
                        "circular_setpoint_range": {
                            "type": "float32",
                            "doc": "circular range in [turns] for position setpoints when circular_setpoints is True"
                        },
                        "steps_per_circular_range": {
                            "type": "int32",
                            "doc": "Number of steps within the circular setpoint range. Set this and the circular setpoint range to powers of 2 for the best results.",
                            "c_setter": "set_steps_per_circular_range"
                        },
                        "homing_speed": {
                            "type": "float32",
                            "unit": "turn/s",
                            "doc": "The speed at which the axis moves towards the `min_endstop` during `AXIS_STATE_HOMING`"
                        },
                        "inertia": {
                            "type": "float32",
                            "unit": "N\u00c2\u00b7m/(turn/s^2)"
                        },
                        "axis_to_mirror": {
                            "type": "uint8",
                            "doc": "The axis used for mirroring when in `INPUT_MODE_MIRROR`"
                        },
                        "mirror_ratio": {
                            "type": "float32",
                            "doc": "The ratio applied to position and velocity movements of the mirrored axis.  To reverse movements, use a negative value"
                        },
                        "torque_mirror_ratio": {
                            "type": "float32",
                            "doc": "The ratio applied to torque values of the mirrored axis."
                        },
                        "load_encoder_axis": {
                            "type": "uint8",
                            "doc": "Default depends on Axis number and is set in load_configuration()"
                        },
                        "input_filter_bandwidth": {
                            "type": "float32",
                            "unit": "rad/s",
                            "c_setter": "set_input_filter_bandwidth",
                            "brief": "The desired bandwidth for `INPUT_MODE_POS_FILTER`.",
                            "doc": "Sets the position filter's P and I gains to emulate a critically-damped 2nd order mass-spring-damper motion."
                        },
                        "anticogging": {
                            "c_is_class": false,
                            "attributes": {
                                "index": "readonly uint32",
                                "pre_calibrated": "bool",
                                "calib_anticogging": "readonly bool",
                                "calib_pos_threshold": "float32",
                                "calib_vel_threshold": "float32",
                                "cogging_ratio": "readonly float32",
                                "anticogging_enabled": "bool"
                            }
                        },
                        "mechanical_power_bandwidth": {
                            "type": "float32",
                            "doc": "Bandwidth for mechanical power estimate. Used for spinout detection",
                            "unit": "rad/s"
                        },
                        "electrical_power_bandwidth": {
                            "type": "float32",
                            "doc": "Bandwidth for electrical power estimate. Used for spinout detection. Dot product of Vdq and Idq",
                            "unit": "rad/s"
                        },
                        "spinout_mechanical_power_threshold": {
                            "type": "float32",
                            "doc": "Mechanical power threshold for spinout detection. This should be a negative value",
                            "unit": "Watt"
                        },
                        "spinout_electrical_power_threshold": {
                            "type": "float32",
                            "doc": "Electrical power threshold for spinout detection. This should be a positive value",
                            "unit": "Watt"
                        }
                    }
                },
                "autotuning": {
                    "c_is_class": false,
                    "doc": "Automatically generate sine waves for frequency-domain response tuning",
                    "attributes": {
                        "frequency": {
                            "type": "float32",
                            "unit": "Hz"
                        },
                        "pos_amplitude": {
                            "type": "float32",
                            "unit": "turns"
                        },
                        "vel_amplitude": {
                            "type": "float32",
                            "unit": "turns/sec"
                        },
                        "torque_amplitude": {
                            "type": "float32",
                            "unit": "N\u00c2\u00b7m"
                        }
                    }
                },
                "mechanical_power": {
                    "type": "readonly float32",
                    "unit": "Watt",
                    "doc": "Mechanical power estimate. Torque * velocity"
                },
                "electrical_power": {
                    "type": "readonly float32",
                    "unit": "Watt",
                    "doc": "Electrical power estimate. Vdq\u00c2\u00b7Idq"
                }
            },
            "functions": {
                "move_incremental": {
                    "doc": "Moves the axes' goal point by a specified increment.",
                    "in": {
                        "displacement": {
                            "type": "float32",
                            "doc": "The desired position change."
                        },
                        "from_input_pos": {
                            "type": "bool",
                            "doc": "If true, the increment is applied relative to `input_pos`. If false, the increment is applied relative to `pos_setpoint`, which usually corresponds roughly to the current position of the axis."
                        }
                    }
                },
                "start_anticogging_calibration": null
            }
        },
        "ODrive.Encoder": {
            "c_is_class": true,
            "attributes": {
                "error": {
                    "nullflag": "NONE",
                    "flags": {
                        "UNSTABLE_GAIN": null,
                        "CPR_POLEPAIRS_MISMATCH": {
                            "doc": "Confirm you have entered the correct count per rotation (CPR) for\n[your encoder](https://docs.odriverobotics.com/encoders). The\nODrive uses your supplied value for the motor pole pairs to\nmeasure the CPR. So you should also double check this value.\n\nIf you are still having issues, you can try to increase\n`config.calib_scan_distance` up to a factor of 4 above the default.\n\nIf your encoder cpr and motor pole pair settings are correct,\nthis error can be caused because motor cogging makes the motor\nmove less or more than commanded. You can fix this by increasing\n`config.calib_scan_distance`.\n\nNote that the AMT encoders are configurable using the micro-\nswitches on the encoder PCB and so you may need to check that\nthese are in the right positions. If your encoder lists its pulse\nper rotation (PPR) multiply that number by four to get CPR.\n"
                        },
                        "NO_RESPONSE": {
                            "doc": "Confirm that your encoder is plugged into the right pins on the\nODrive board.\n"
                        },
                        "UNSUPPORTED_ENCODER_MODE": null,
                        "ILLEGAL_HALL_STATE": {
                            "doc": "Hall effect encoder only have 6 valid states out of 8 (2^3) possible states.\nAn invalid state can be caused by noise or a hardware fault. If you get this\nerror and you are sure that your electrical connections are correct, add\n22nF capacitors between the encoder A,B,Z pins and ground to filter out noise.\n"
                        },
                        "INDEX_NOT_FOUND_YET": {
                            "doc": "Check that your encoder is a model that has an index pulse. If\nyour encoder does not have a wire connected to pin Z on your\nODrive then it does not output an index pulse.\n"
                        },
                        "ABS_SPI_TIMEOUT": null,
                        "ABS_SPI_COM_FAIL": null,
                        "ABS_SPI_NOT_READY": null,
                        "HALL_NOT_CALIBRATED_YET": null
                    }
                },
                "is_ready": "readonly bool",
                "index_found": "readonly bool",
                "shadow_count": {
                    "type": "readonly int32",
                    "unit": "counts",
                    "doc": "Raw linear count from the encoder."
                },
                "count_in_cpr": {
                    "type": "readonly int32",
                    "unit": "counts",
                    "doc": "Raw circular count from the encoder on [0, cpr)"
                },
                "interpolation": "readonly float32",
                "phase": {
                    "type": "readonly float32",
                    "c_getter": "phase_.any().value_or(0.0f)"
                },
                "pos_estimate": {
                    "type": "readonly float32",
                    "c_getter": "pos_estimate_.any().value_or(0.0f)",
                    "unit": "turns",
                    "doc": "Linear position estimate of the encoder, in turns.  Also known as \"multi-turn\" position."
                },
                "pos_estimate_counts": {
                    "type": "readonly float32",
                    "unit": "counts",
                    "doc": "Linear position estimate of the encoder, in counts.  Equal to `pos_estimate * config.cpr`"
                },
                "pos_circular": {
                    "type": "readonly float32",
                    "c_getter": "pos_circular_.any().value_or(0.0f)",
                    "unit": "turns",
                    "doc": "Circular position estimate of the encoder, as a decimal from [0, 1).  Also known as \"single-turn\" position."
                },
                "pos_cpr_counts": {
                    "type": "readonly float32",
                    "unit": "counts",
                    "doc": "Circular position estimate of the encoder, on the space [0, cpr)."
                },
                "delta_pos_cpr_counts": {
                    "type": "readonly float32",
                    "unit": "counts",
                    "doc": "Circular position delta of the encoder in the most recent loop. Primarily for debug purposes, it indicates much the encoder changed since the last time it was checked."
                },
                "hall_state": "readonly uint8",
                "vel_estimate": {
                    "type": "readonly float32",
                    "c_getter": "vel_estimate_.any().value_or(0.0f)",
                    "unit": "turn/s",
                    "doc": "Estimate of the linear velocity of an axis in turn/s"
                },
                "vel_estimate_counts": {
                    "type": "readonly float32",
                    "unit": "counts/sec",
                    "doc": "Estimate of the linear velocity of an axis, in counts/s."
                },
                "calib_scan_response": "readonly float32",
                "pos_abs": {
                    "type": "int32",
                    "doc": "The last (valid) position from an absolute encoder, if used."
                },
                "spi_error_rate": "readonly float32",
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "mode": "Mode",
                        "use_index": {
                            "type": "bool",
                            "c_setter": "set_use_index",
                            "doc": "Set this to `True` when using an encoder with an index pin to allow pre-calibration of the encoder and encoder index search."
                        },
                        "index_offset": {
                            "type": "float32",
                            "doc": "When the index is found, the linear position of the axis will be set to this value if `use_index_offset` is set to `True`."
                        },
                        "use_index_offset": {
                            "type": "bool",
                            "doc": "Set this to `True` to have the axis set the linear position of the axis to `index_offset` when the index is found during encoder index search"
                        },
                        "find_idx_on_lockin_only": {
                            "type": "bool",
                            "c_setter": "set_find_idx_on_lockin_only"
                        },
                        "abs_spi_cs_gpio_pin": {
                            "type": "uint16",
                            "c_setter": "set_abs_spi_cs_gpio_pin",
                            "doc": "Make sure that the GPIO is in `GPIO_MODE_DIGITAL`."
                        },
                        "cpr": {
                            "type": "int32",
                            "doc": "Counts per Revolution of the encoder.  This is 4x the Pulses per Revolution."
                        },
                        "phase_offset": "int32",
                        "phase_offset_float": "float32",
                        "direction": "int32",
                        "pre_calibrated": {
                            "type": "bool",
                            "c_setter": "set_pre_calibrated"
                        },
                        "enable_phase_interpolation": "bool",
                        "bandwidth": {
                            "type": "float32",
                            "c_setter": "set_bandwidth",
                            "unit": "rad/s"
                        },
                        "calib_range": {
                            "type": "float32",
                            "unit": "turn",
                            "doc": "The error threshold for encoder offset calibration, in turns.  If the"
                        },
                        "calib_scan_distance": {
                            "type": "float32",
                            "unit": "rad (electrical)",
                            "doc": "The distance the motor is turned during encoder offset calibration.\nThis is in the electrical frame, so if you want a 7pp motor to turn 1 mechanical revolution, you would put `(7 * 2*pi)`\n",
                            "default": "16pi rad"
                        },
                        "calib_scan_omega": {
                            "type": "float32",
                            "unit": "rad/s (electrical)",
                            "doc": "The speed the motor turns during encoder offset calibration.\nThis is in the electrical frame, so a 7pp motor rotating 1 mechanical revolution in 2 sec would be set to `(7 * 2*pi) / 2`\n",
                            "default": "4pi rad/s"
                        },
                        "ignore_illegal_hall_state": {
                            "type": "bool",
                            "doc": "Ignore the error \"Illegal Hall State\""
                        },
                        "hall_polarity": "uint8",
                        "hall_polarity_calibrated": "bool",
                        "sincos_gpio_pin_sin": {
                            "type": "uint16",
                            "doc": "Analog sine signal of a sin/cos encoder. The corresponding GPIO must be in `GPIO_MODE_ANALOG_IN`."
                        },
                        "sincos_gpio_pin_cos": {
                            "type": "uint16",
                            "doc": "Analog cosine signal of a sin/cos encoder. The corresponding GPIO must be in `GPIO_MODE_ANALOG_IN`."
                        }
                    }
                }
            },
            "functions": {
                "set_linear_count": {
                    "in": {
                        "count": {
                            "type": "int32",
                            "unit": "counts"
                        }
                    },
                    "doc": "Set the current linear position of the axis, in counts"
                }
            }
        },
        "ODrive.SensorlessEstimator": {
            "c_is_class": true,
            "attributes": {
                "error": {
                    "nullflag": "NONE",
                    "flags": {
                        "UNSTABLE_GAIN": null,
                        "UNKNOWN_CURRENT_MEASUREMENT": null
                    }
                },
                "phase": {
                    "type": "readonly float32",
                    "unit": "rad",
                    "c_getter": "phase_.any().value_or(0.0f)"
                },
                "pll_pos": {
                    "type": "readonly float32",
                    "unit": "rad"
                },
                "phase_vel": {
                    "type": "readonly float32",
                    "unit": "rad/s",
                    "c_getter": "phase_vel_.any().value_or(0.0f)"
                },
                "vel_estimate": {
                    "type": "readonly float32",
                    "unit": "turn/s",
                    "c_getter": "vel_estimate_.any().value_or(0.0f)"
                },
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "observer_gain": "float32",
                        "pll_bandwidth": "float32",
                        "pm_flux_linkage": "float32"
                    }
                }
            }
        },
        "ODrive.TrapezoidalTrajectory": {
            "c_is_class": true,
            "attributes": {
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "vel_limit": {
                            "type": "float32",
                            "unit": "turn/s"
                        },
                        "accel_limit": {
                            "type": "float32",
                            "unit": "turn/s^2"
                        },
                        "decel_limit": {
                            "type": "float32",
                            "unit": "turn/s^2"
                        }
                    }
                }
            }
        },
        "ODrive.Endstop": {
            "c_is_class": true,
            "attributes": {
                "endstop_state": "readonly bool",
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "gpio_num": {
                            "type": "uint16",
                            "c_setter": "set_gpio_num",
                            "doc": "Make sure the corresponding GPIO is in `GPIO_MODE_DIGITAL`."
                        },
                        "enabled": {
                            "type": "bool",
                            "c_setter": "set_enabled"
                        },
                        "offset": {
                            "type": "float32",
                            "unit": "turns"
                        },
                        "is_active_high": "bool",
                        "debounce_ms": {
                            "type": "uint32",
                            "c_setter": "set_debounce_ms"
                        }
                    }
                }
            }
        },
        "ODrive.MechanicalBrake": {
            "c_is_class": true,
            "attributes": {
                "config": {
                    "c_is_class": false,
                    "attributes": {
                        "gpio_num": {
                            "type": "uint16",
                            "c_setter": "set_gpio_num"
                        },
                        "is_active_low": "bool"
                    }
                }
            },
            "functions": {
                "engage": {
                    "doc": "This function engages the mechanical brake if one is present and enabled.\n"
                },
                "release": {
                    "doc": "This function releases the mecahncal brake if one is present and enabled.\n"
                }
            }
        },
        "ODrive.TaskTimer": {
            "c_is_class": true,
            "attributes": {
                "start_time": "readonly uint32",
                "end_time": "readonly uint32",
                "length": "readonly uint32",
                "max_length": "uint32"
            }
        },
        "ODrive3": {
            "c_is_class": true,
            "implements": "ODrive",
            "attributes": {
                "config": {
                    "c_is_class": false,
                    "implements": "ODrive.Config",
                    "attributes": {
                        "gpio1_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO1 (changes take effect after reboot)",
                            "c_name": "gpio_modes[1]"
                        },
                        "gpio2_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO2 (changes take effect after reboot)",
                            "c_name": "gpio_modes[2]"
                        },
                        "gpio3_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO3 (changes take effect after reboot)",
                            "c_name": "gpio_modes[3]"
                        },
                        "gpio4_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO4 (changes take effect after reboot)",
                            "c_name": "gpio_modes[4]"
                        },
                        "gpio5_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO5 (changes take effect after reboot)",
                            "c_name": "gpio_modes[5]"
                        },
                        "gpio6_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO6 (changes take effect after reboot)",
                            "c_name": "gpio_modes[6]"
                        },
                        "gpio7_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO7 (changes take effect after reboot)",
                            "c_name": "gpio_modes[7]"
                        },
                        "gpio8_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO8 (changes take effect after reboot)",
                            "c_name": "gpio_modes[8]"
                        },
                        "gpio9_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO9 (changes take effect after reboot)",
                            "c_name": "gpio_modes[9]"
                        },
                        "gpio10_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO10 (changes take effect after reboot)",
                            "c_name": "gpio_modes[10]"
                        },
                        "gpio11_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO11 (changes take effect after reboot)",
                            "c_name": "gpio_modes[11]"
                        },
                        "gpio12_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO12 (changes take effect after reboot)",
                            "c_name": "gpio_modes[12]"
                        },
                        "gpio13_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO13 (changes take effect after reboot)",
                            "c_name": "gpio_modes[13]"
                        },
                        "gpio14_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO14 (changes take effect after reboot)",
                            "c_name": "gpio_modes[14]"
                        },
                        "gpio15_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO15 (changes take effect after reboot)",
                            "c_name": "gpio_modes[15]"
                        },
                        "gpio16_mode": {
                            "type": "ODrive.GpioMode",
                            "doc": "Mode of GPIO16 (changes take effect after reboot)",
                            "c_name": "gpio_modes[16]"
                        },
                        "gpio1_pwm_mapping": {
                            "type": "ODrive.Endpoint",
                            "c_name": "pwm_mappings[0]",
                            "doc": "Make sure the corresponding GPIO is in `GPIO_MODE_PWM`."
                        },
                        "gpio2_pwm_mapping": {
                            "type": "ODrive.Endpoint",
                            "c_name": "pwm_mappings[1]",
                            "doc": "Make sure the corresponding GPIO is in `GPIO_MODE_PWM`."
                        },
                        "gpio3_pwm_mapping": {
                            "type": "ODrive.Endpoint",
                            "c_name": "pwm_mappings[2]",
                            "doc": "Make sure the corresponding GPIO is in `GPIO_MODE_PWM`."
                        },
                        "gpio4_pwm_mapping": {
                            "type": "ODrive.Endpoint",
                            "c_name": "pwm_mappings[3]",
                            "doc": "Make sure the corresponding GPIO is in `GPIO_MODE_PWM`."
                        }
                    }
                },
                "axis0": {
                    "type": "ODrive.Axis",
                    "c_name": "get_axis(0)"
                },
                "axis1": {
                    "type": "ODrive.Axis",
                    "c_name": "get_axis(1)"
                }
            }
        }
    },
    "valuetypes": {
        "ODrive.GpioMode": {
            "values": {
                "DIGITAL": {
                    "doc": "The pin can be used for one or more of these functions:\nStep, dir, enable, encoder index, hall effect encoder, SPI encoder nCS (this one is exclusive).\n"
                },
                "DIGITAL_PULL_UP": {
                    "doc": "Same as `DIGITAL` but with the internal pull-up resistor enabled."
                },
                "DIGITAL_PULL_DOWN": {
                    "doc": "Same as `DIGITAL` but with the internal pull-down resistor enabled."
                },
                "ANALOG_IN": {
                    "doc": "The pin can be used for one or more of these functions:\nSin/cos encoders, analog input, `get_adc_voltage()`.\n"
                },
                "UART_A": {
                    "doc": "See `config.enable_uart_a`."
                },
                "UART_B": {
                    "doc": "This mode is not supported on ODrive v3.x."
                },
                "UART_C": {
                    "doc": "This mode is not supported on ODrive v3.x."
                },
                "CAN_A": {
                    "doc": "See `config.enable_can_a`."
                },
                "I2C_A": {
                    "doc": "See `config.enable_i2c_a`."
                },
                "SPI_A": {
                    "doc": "Note that the SPI pins on ODrive v3.x are hardwired so they cannot be configured through software. Consequently",
                    "even though SPI_A is exposed": null,
                    "this mode is of no use on ODrive v3.x.": null
                },
                "PWM": {
                    "doc": "See `config.gpio1_pwm_mapping`."
                },
                "ENC0": {
                    "doc": "The pin is used by quadrature encoder 0."
                },
                "ENC1": {
                    "doc": "The pin is used by quadrature encoder 1."
                },
                "ENC2": {
                    "doc": "This mode is not supported on ODrive v3.x."
                },
                "MECH_BRAKE": {
                    "doc": "This is to support external mechanical brakes."
                },
                "STATUS": {
                    "doc": "The pin is used for status output (see `config.error_gpio_pin`)"
                }
            }
        },
        "ODrive.StreamProtocolType": {
            "values": {
                "Fibre": {
                    "doc": "Machine-to-machine protocol which gives access to all features of the\nODrive. This protocol is used by the official odrivetool and GUI.\nDevelopers who wish to interact with this protocol are advised to do\nso through libfibre.\n"
                },
                "Ascii": {
                    "doc": "Human readable protocol designed for easy implementation for cases\nwhere the use of libfibre is not desired or feasible.\nRefer to [this page](ascii-protocol.md) for details.\n"
                },
                "Stdout": {
                    "doc": "Output of printf(). Only intended for developers who modify ODrive firmware."
                },
                "AsciiAndStdout": {
                    "doc": "Combination of `Ascii` and `Stdout`."
                }
            }
        },
        "ODrive.Can.Protocol": {
            "flags": {
                "SIMPLE": {
                    "doc": "CANSimple, an ODrive-specific protocol for basic functionality"
                }
            }
        },
        "ODrive.Axis.AxisState": {
            "values": {
                "UNDEFINED": {
                    "doc": "will fall through to idle"
                },
                "IDLE": {
                    "brief": "Disable motor PWM and do nothing."
                },
                "STARTUP_SEQUENCE": {
                    "brief": "Run the startup procedure.",
                    "doc": "the actual sequence is defined by the `config`.startup... flags"
                },
                "FULL_CALIBRATION_SEQUENCE": {
                    "doc": "Run motor calibration and then encoder offset calibration (or encoder index search if `<axis>.encoder.config.use_index` is `True`)."
                },
                "MOTOR_CALIBRATION": {
                    "brief": "Measure phase resistance and phase inductance of the motor.",
                    "doc": "* To store the results set `motor.config.pre_calibrated` to `True`\nand save the configuration (`save_configuration()`). After that you\ndon't have to run the motor calibration on the next start up.\n* This modifies the variables `motor.config.phase_resistance` and\n`motor.config.phase_inductance`.\n"
                },
                "ENCODER_INDEX_SEARCH": {
                    "brief": "Turn the motor in one direction until the encoder index is traversed.",
                    "doc": "This state can only be entered if `encoder.config.use_index` is `True`.",
                    "value": 6
                },
                "ENCODER_OFFSET_CALIBRATION": {
                    "brief": "Turn the motor in one direction for a few seconds and then back to measure the offset between the encoder position and the electrical phase.",
                    "doc": "* Can only be entered if the motor is calibrated (`motor.is_calibrated`).\n* A successful encoder calibration will make the `encoder.is_ready`\ngo to true.\n"
                },
                "CLOSED_LOOP_CONTROL": {
                    "brief": "Run closed loop control.",
                    "doc": "* The action depends on the `controller.config.control_mode`.\n* Can only be entered if the motor is calibrated\n(`motor.is_calibrated`) and the encoder is ready (`encoder.is_ready`).\n"
                },
                "LOCKIN_SPIN": {
                    "brief": "Run lockin spin.",
                    "doc": "Can only be entered if the motor is calibrated (`motor.is_calibrated`)\nor the motor direction is unspecified (`encoder.config.direction` == 1)\n"
                },
                "ENCODER_DIR_FIND": {
                    "brief": "Run encoder direction search.",
                    "doc": "Can only be entered if the motor is calibrated (`motor.is_calibrated`).\n"
                },
                "HOMING": {
                    "brief": "Run axis homing function.",
                    "doc": "Endstops must be enabled to use this feature."
                },
                "ENCODER_HALL_POLARITY_CALIBRATION": {
                    "brief": "Rotate the motor in lockin and calibrate hall polarity",
                    "doc": "ODrive assumes 120 degree electrical hall spacing. This routine determines if that is the case and sets the polarity if the halls are on 60 degree electrical spacing"
                },
                "ENCODER_HALL_PHASE_CALIBRATION": {
                    "brief": "Rotate the motor for 30s to calibrate hall sensor edge offsets",
                    "doc": "The phase offset is not calibrated at this time, so the map is only relative"
                }
            }
        },
        "ODrive.Encoder.Mode": {
            "values": {
                "INCREMENTAL": null,
                "HALL": null,
                "SINCOS": null,
                "SPI_ABS_CUI": {
                    "value": 256,
                    "doc": "Compatible with CUI AMT23xx"
                },
                "SPI_ABS_AMS": {
                    "value": 257,
                    "doc": "Compatible with AMS AS5047P, AS5048A/AS5048B (no daisy chain support)"
                },
                "SPI_ABS_AEAT": {
                    "value": 258,
                    "doc": "Supports AEAT-8800"
                },
                "SPI_ABS_RLS": {
                    "value": 259,
                    "doc": "Supports RLS Orbis Encoders"
                },
                "SPI_ABS_MA732": {
                    "value": 260,
                    "doc": "MagAlpha MA732 magnetic encoder"
                }
            }
        },
        "ODrive.Controller.ControlMode": {
            "values": {
                "VOLTAGE_CONTROL": {
                    "doc": "Note: This mode is not used internally.  For voltage-only FOC, use `MotorType.GIMBAL`\n"
                },
                "TORQUE_CONTROL": {
                    "doc": "Uses only the inner torque control loop.\nUse `input_torque` to command desired torque.\nNote the setting `motor.config.torque_constant`.\nNote the setting `enable_torque_mode_vel_limit`.\n"
                },
                "VELOCITY_CONTROL": {
                    "doc": "Uses both the inner torque control loop and the velocity control loop.\nUse `input_vel` to command desired velocity, and `input_torque`.\n"
                },
                "POSITION_CONTROL": {
                    "doc": "Uses the inner torque loop, the velocity control loop, and the outer position control loop.\nUse `input_pos` to command desired position, `input_vel` to command velocity feed-forward, and `input_torque` for torque feed-forward.\n"
                }
            }
        },
        "ODrive.Controller.InputMode": {
            "values": {
                "INACTIVE": {
                    "brief": "Disable inputs. Setpoints retain their last value."
                },
                "PASSTHROUGH": {
                    "brief": "Pass `input_xxx` through to `xxx_setpoint` directly.",
                    "doc": "### Valid Inputs:\n* `input_pos`\n* `input_vel`\n* `input_torque`\n\n### Valid Control modes:\n* `CONTROL_MODE_VOLTAGE_CONTROL`\n* `CONTROL_MODE_TORQUE_CONTROL`\n* `CONTROL_MODE_VELOCITY_CONTROL`\n* `CONTROL_MODE_POSITION_CONTROL`\n"
                },
                "VEL_RAMP": {
                    "brief": "Ramps a velocity command from the current value to the target value.",
                    "doc": "### Configuration Values:\n* `config.vel_ramp_rate` [turn/s]\n* `config.inertia` [N\u00c2\u00b7m/(turn/s^2))]\n\n### Valid inputs:\n* `input_vel`\n\n### Valid Control Modes:\n* `CONTROL_MODE_VELOCITY_CONTROL`\n"
                },
                "POS_FILTER": {
                    "brief": "Implements a 2nd order position tracking filter.",
                    "doc": "Intended for use with step/dir interface, but can also be used with\nposition-only commands.\n\n![POS Filter Response](../secondOrderResponse.PNG)\nResult of a step command from 1000 to 0\n\n### Configuration Values:\n* `config.input_filter_bandwidth`\n* `config.inertia`\n\n### Valid inputs:\n* `input_pos`\n\n### Valid Control modes:\n* `CONTROL_MODE_POSITION_CONTROL`\n"
                },
                "MIX_CHANNELS": {
                    "brief": "Not Implemented."
                },
                "TRAP_TRAJ": {
                    "brief": "Implementes an online trapezoidal trajectory planner.",
                    "doc": "![Trapezoidal Planner Response](../TrapTrajPosVel.PNG)\n\n### Configuration Values:\n* `Axis:trap_traj.config.vel_limit`\n* `Axis:trap_traj.config.accel_limit`\n* `Axis:trap_traj.config.decel_limit`\n* `config.inertia`\n\n### Valid Inputs:\n* `input_pos`\n\n### Valid Control Modes:\n* `CONTROL_MODE_POSITION_CONTROL`\n"
                },
                "TORQUE_RAMP": {
                    "brief": "Ramp a torque command from the current value to the target value.",
                    "doc": "### Configuration Values:\n* `config.torque_ramp_rate`\n\n### Valid Inputs:\n* `input_torque`\n\n### Valid Control Modes:\n* `CONTROL_MODE_TORQUE_CONTROL`\n"
                },
                "MIRROR": {
                    "brief": "Implements \"electronic mirroring\".",
                    "doc": "This is like electronic camming, but you can only mirror exactly the\nmovements of the other motor, according to a fixed ratio.\n\n[![](http://img.youtube.com/vi/D4_vBtyVVzM/0.jpg)](http://www.youtube.com/watch?v=D4_vBtyVVzM \"Example Mirroring Video\")\n\n### Configuration Values\n* `config.axis_to_mirror`\n* `config.mirror_ratio`\n\n### Valid Inputs\n* None.  Inputs are taken directly from the other axis encoder estimates\n\n### Valid Control modes\n* `CONTROL_MODE_POSITION_CONTROL`\n* `CONTROL_MODE_VELOCITY_CONTROL`\n* `CONTROL_MODE_TORQUE_CONTROL`\n"
                },
                "TUNING": {
                    "brief": "Implements a tuning mode",
                    "doc": "Used for tuning your odrive, this mode allows the user to set different frequencies.\nSet control_mode for the loop you want to tune, then set the frequency desired.\nThe ODrive will send a 1 turn amplitude sine wave to the controller with the given frequency and phase.\n"
                }
            }
        },
        "ODrive.Motor.MotorType": {
            "values": {
                "HIGH_CURRENT": {
                    "doc": "Used for all standard \"hobby-style\" motors Permanant Magnet AC (PMAC) or Brushless DC (BLDC) motors.  This is the default.\nNote: Assumes sinusoidal back-EMF, \"wye\" connected motors.  Trapezoidal bEMF may have reduced controllability.\n"
                },
                "GIMBAL": {
                    "value": 2,
                    "doc": "Enables voltage-only FOC where V=IR can be assumed to be correct.\nUsed for high-phase-resistance motors (> 1 ohm), which are typically sold as \"Gimbal\" motors.\n"
                },
                "ACIM": {
                    "doc": "Used for FOC control of AC Induction Motors (ACIM), aka Asynchronous motors, \n"
                }
            }
        }
    }
}