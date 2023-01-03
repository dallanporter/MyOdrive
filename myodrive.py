
@classproperty_support
class MyOdrive():
    error = None
    vbus_voltage = None
    ibus = None
    type = None
    
    _ibus_report_filter_k = None
    
    @classproperty
    def ibus_report_filter_k(self, value):
        self._ibus_report_filter_k = value
        
    serial_number = None
    hw_version_major = None
    hw_version_minor = None
    hw_version_variant = None
    fw_version_major = None
    fw_version_minor = None
    fw_version_revision = None
    fw_version_unreleased = None
    brake_resistor_armed = None
    brake_resistor_saturated = None
    brake_resistor_current = None
    n_evt_sampling = None
    n_evt_control_loop = None
    task_timers_armed = None
    task_times = None
    system_stats = {
        'uptime': None,
        'min_heap_space': None,
        'max_stack_usage_axis': None,
        'max_stack_usage_usb': None,
        'max_stack_usage_uart': None,
        'max_stack_usage_can': None,
        'max_stack_usage_startup': None,
        'max_stack_usage_analog': None,
        'stack_size_axis': None,
        'stack_size_usb': None,
        'stack_size_uart': None,
        'stack_size_startup': None,
        'stack_size_can': None,
        'stack_size_analog': None,
        'prio_axis': None,
        'prio_usb': None,
        'prio_uart': None,
        'prio_startup': None,
        'prio_can': None,
        'prio_analog': None,
        'usb': {                                                
                'rx_cnt': None,
                'tx_cnt': None,
                'tx_overrun_cnt': None                    
        },
        'i2c': {                                                
                'addr': None,
                'addr_match_cnt': None,
                'rx_cnt': None,
                'error_cnt': None,
        }
    }
    user_config_loaded = None
    misconfigured = None
    oscilloscope = None
    can = None
    test_property = None
    otp_valid = None
    
    def __init__(self):
        # MyOdrive constructor
        pass
    
    def test_function():
        pass
    
    def get_adc_voltage():
        pass
    
    def save_configuration():
        pass
    
    def erase_configuration():
        pass
    
    def reboot(): 
        pass
    
    def enter_dfu_mode(): 
        pass
    
    def get_interrupt_status():
        pass
    
    def get_dma_status():
        pass
    
    def get_gpio_states():
        pass
    
    def get_drv_fault():
        pass
    
    def clear_errors():
        pass
    

class Config():
    enable_uart_a = None
    enable_uart_b = None
    enable_uart_c = None
    uart_a_baudrate = None
    uart_b_baudrate = None
    uart_c_baudrate = None
    enable_can_a = None
    enable_i2c_a = None
    usb_cdc_protocol = None
    uart0_protocol = None
    uart1_protocol = None
    uart2_protocol = None
    max_regen_current = None
    brake_resistance = None
    enable_brake_resistor = None
    dc_bus_undervoltage_trip_level = None
    dc_bus_overvoltage_trip_level = None
    enable_dc_bus_overvoltage_ramp = None
    dc_bus_overvoltage_ramp_start = None
    dc_bus_overvoltage_ramp_end = None
    dc_max_positive_current = None
    dc_max_negative_current = None
    error_gpio_pin = None
    gpio3_analog_mapping = None
    gpio4_analog_mapping = None


class Can():
    error = None
    config = None


class Endpoint():
    endpoint = None
    min = None
    max = None


class Axis():
    error = None
    step_dir_active = None
    last_drv_fault = None
    steps = None
    current_state = None
    requested_state = None
    is_homed = None
    config = {
        'startup_motor_calibration': None,
        'startup_encoder_index_search': None,
        'startup_encoder_offset_calibration': None,
        'startup_closed_loop_control': None,
        'startup_homing': None,
        'enable_step_dir': None,
        'step_dir_always_on': None,
        'enable_sensorless_mode': None,
        'watchdog_timeout':None,
        'enable_watchdog':None,
        'step_gpio_pin': None,
        'dir_gpio_pin': None,
        'calibration_lockin': {
            'current': None,
            'ramp_time': None,
            'ramp_distance': None,
            'accel': None,
            'vel': None
        },
        'sensorless_ramp': None,
        'general_lockin': None,
        'can': None
    }
    motor:Motor = None
    controller = None
    encoder = None
    acim_estimator = None
    sensorless_estimator = None
    trap_traj = None
    min_endstop = None
    max_endstop = None
    mechanical_brake = None
    task_times = None
    thermistor_update = None
    encoder_update = None
    sensorless_estimator_update = None
    endstop_update = None
    can_heartbeat = None
    controller_update = None
    open_loop_controller_update = None
    acim_estimator_update = None
    motor_update = None
    current_controller_update = None
    dc_calib = None
    current_sense = None
    pwm_update = None

    def watchdog_feed():
        pass



class LockinConfig():
    current = None
    ramp_time = None
    ramp_distance = None
    accel = None
    vel = None
    finish_distance = None
    finish_on_vel = None
    finish_on_distance = None
    finish_on_enc_idx = None


class CanConfig():
    node_id = None
    is_extended = None
    heartbeat_rate_ms = None
    encoder_rate_ms = None

class OnboardThermistorCurrentLimiter():
    temperature = None
    config = {
        'temp_limit_lower': None,
        'temp_limit_upper': None,
        'enabled': None
    }

class OffboardThermistorCurrentLimiter():
    temperature = None
    config = {
        'gpio_pin': None,
        'poly_coefficient_0': None,
        'poly_coefficient_1': None,
        'poly_coefficient_2': None,
        'poly_coefficient_3': None,
        'temp_limit_lower': None,
        'temp_limit_upper': None,
        'enabled': None
    }

class Motor():
    last_error_time = None
    error= None
    is_armed = None
    is_calibrated = None
    current_meas_phA = None
    current_meas_phB = None
    current_meas_phC = None
    DC_calib_phA = None
    DC_calib_phB = None
    DC_calib_phC = None
    I_bus = None
    phase_current_rev_gain = None
    effective_current_lim = None
    max_allowed_current = None
    max_dc_calib = None
    fet_thermistor = None
    motor_thermistor = None
    current_control = {
        'p_gain': None,
        'i_gain': None,
        'I_measured_report_filter_k': None,
        'Id_setpoint': None,
        'Iq_setpoint': None,
        'Vd_setpoint': None,
        'Vq_setpoint': None,
        'phase': None,
        'phase_vel': None,
        'Ialpha_measured': None,
        'Ibeta_measured': None,
        'Id_measured': None,
        'Iq_measured': None,
        'power': None,
        'v_current_control_integral_d': None,
        'v_current_control_integral_q': None,
        'final_v_alpha': None,
        'final_v_beta': None
    }
    n_evt_current_measurement = None
    n_evt_pwm_update = None
    config = {
        'pre_calibrated': None,
        'pole_pairs': None,
        'calibration_current': None,
        'resistance_calib_max_voltage': None,
        'phase_inductance': None,
        'phase_resistance': None,                           
        'torque_constant': None,
        'motor_type': None,
        'current_lim': None,
        'current_lim_margin': None,
        'torque_lim': None,
        'inverter_temp_limit_lower': None,
        'inverter_temp_limit_upper': None,
        'requested_current_range': None,
        'current_control_bandwidth': None,
        'acim_gain_min_flux': None,
        'acim_autoflux_min_Id': None,
        'acim_autoflux_enable': None,
        'acim_autoflux_attack_gain': None,
        'acim_autoflux_decay_gain': None,
        'R_wL_FF_enable': None,
        'bEMF_FF_enable': None,
        'I_bus_hard_min': None,
        'I_bus_hard_max': None,
        'I_leak_max': None,
        'dc_calib_tau': None
    }


class Oscilloscope():
    size = None
    def get_val():
        return {
            'in': {
                'index': None
                },
            'out': {
                'val': None
            }
        }
            

class AcimEstimator():
    rotor_flux = None
    slip_vel = None
    phase_offset = None
    stator_phase_vel = None
    stator_phase = None
    config = None
    slip_velocity = None


class Controller():
    error = None
    last_error_time = None
    input_pos = None
    input_vel = None
    input_torque = None
    pos_setpoint = None
    vel_setpoint = None
    torque_setpoint = None
    trajectory_done = None
    vel_integrator_torque = None
    anticogging_valid = None
    autotuning_phase = None
    config = {
        'gain_scheduling_width': None,
        'enable_vel_limit': None,
        'enable_torque_mode_vel_limit': None,
        'enable_gain_scheduling': None,
        'enable_overspeed_error': None,
        'control_mode': None,
        'input_mode': None,
        'pos_gain': None,
        'vel_gain': None,
        'vel_integrator_gain': None,
        'vel_integrator_limit': None,
        'vel_limit': None,
        'vel_limit_tolerance': None,
        'vel_ramp_rate': None,
        'torque_ramp_rate': None,
        'circular_setpoints': None,
        'circular_setpoint_range': None,
        'steps_per_circular_range': None,
        'homing_speed': None,
        'inertia': None,
        'axis_to_mirror': None,
        'mirror_ratio': None,
        'torque_mirror_ratio': None,
        'load_encoder_axis': None,
        'input_filter_bandwidth': None,
        'anticogging': {
            'index': None,
            'pre_calibrated': None,
            'calib_anticogging': None,
            'calib_pos_threshold': None,
            'calib_vel_threshold': None,
            'cogging_ratio': None,
            'anticogging_enabled': None
        },
        'mechanical_power_bandwidth': None,
        'electrical_power_bandwidth': None,
        'spinout_mechanical_power_threshold': None,
        'spinout_electrical_power_threshold': None,
    }
    autotuning = {
        'frequency': None,
        'pos_amplitude': None,
        'vel_amplitude': None,
        'torque_amplitude': None
    }
    mechanical_power = None
    electrical_power = None
    
    
    def move_incremental():
        pass
    
    def start_anticogging_calibration():
        pass


        "ODrive.Encoder": {

                error
                    nullflag
                    flags
                        UNSTABLE_GAIN
                        CPR_POLEPAIRS_MISMATCH

                        NO_RESPONSE

                        UNSUPPORTED_ENCODER_MODE
                        ILLEGAL_HALL_STATE

                        INDEX_NOT_FOUND_YET

                        ABS_SPI_TIMEOUT
                        ABS_SPI_COM_FAIL
                        ABS_SPI_NOT_READY
                        HALL_NOT_CALIBRATED_YET


                is_ready
                index_found
                shadow_count

                count_in_cpr

                interpolation
                phase
                    

                pos_estimate
                    

                pos_estimate_counts

                pos_circular
                    

                pos_cpr_counts

                delta_pos_cpr_counts

                hall_state
                vel_estimate
                    

                vel_estimate_counts

                calib_scan_response
                pos_abs

                spi_error_rate
                config

                        mode
                        use_index
                            

                        index_offset

                        use_index_offset

                        find_idx_on_lockin_only
                            

                        abs_spi_cs_gpio_pin
                            

                        cpr

                        phase_offset
                        phase_offset_float
                        direction
                        pre_calibrated
                            

                        enable_phase_interpolation
                        bandwidth
                            

                        calib_range

                        calib_scan_distance
                            default

                        calib_scan_omega
                            default

                        ignore_illegal_hall_state

                        hall_polarity
                        hall_polarity_calibrated
                        sincos_gpio_pin_sin

                        sincos_gpio_pin_cos




            functions
                set_linear_count
                    in
                        count





        "ODrive.SensorlessEstimator": {

                error
                    nullflag
                    flags
                        UNSTABLE_GAIN
                        UNKNOWN_CURRENT_MEASUREMENT


                phase
                    

                pll_pos

                phase_vel
                    

                vel_estimate
                    

                config

                        observer_gain
                        pll_bandwidth
                        pm_flux_linkage




        "ODrive.TrapezoidalTrajectory": {

                config

                        vel_limit

                        accel_limit

                        decel_limit





        "ODrive.Endstop": {

                endstop_state
                config

                        gpio_num
                            

                        enabled
                            

                        offset

                        is_active_high
                        debounce_ms
                            





        "ODrive.MechanicalBrake": {

                config

                        gpio_num
                            

                        is_active_low



            functions
                engage

                release



        "ODrive.TaskTimer": {

                start_time
                end_time
                length
                max_length


        ODrive3

            implements
                config

                    implements
                        gpio1_mode


                        gpio2_mode


                        gpio3_mode


                        gpio4_mode


                        gpio5_mode


                        gpio6_mode


                        gpio7_mode


                        gpio8_mode


                        gpio9_mode


                        gpio10_mode


                        gpio11_mode


                        gpio12_mode


                        gpio13_mode


                        gpio14_mode


                        gpio15_mode


                        gpio16_mode


                        gpio1_pwm_mapping


                        gpio2_pwm_mapping


                        gpio3_pwm_mapping


                        gpio4_pwm_mapping




                axis0


                axis1





    valuetypes
        "ODrive.GpioMode": {
            values
                DIGITAL

                DIGITAL_PULL_UP

                DIGITAL_PULL_DOWN

                ANALOG_IN

                UART_A

                UART_B

                UART_C

                CAN_A

                I2C_A

                SPI_A
                    "even though SPI_A is exposed": null,
                    "this mode is of no use on ODrive v3.x.": null

                PWM

                ENC0

                ENC1

                ENC2

                MECH_BRAKE

                STATUS



        "ODrive.StreamProtocolType": {
            values
                Fibre

                Ascii

                Stdout

                AsciiAndStdout



        "ODrive.Can.Protocol": {
            flags
                SIMPLE



        "ODrive.Axis.AxisState": {
            values
                UNDEFINED

                IDLE

                STARTUP_SEQUENCE

                FULL_CALIBRATION_SEQUENCE

                MOTOR_CALIBRATION

                ENCODER_INDEX_SEARCH
                    value

                ENCODER_OFFSET_CALIBRATION

                CLOSED_LOOP_CONTROL

                LOCKIN_SPIN

                ENCODER_DIR_FIND

                HOMING

                ENCODER_HALL_POLARITY_CALIBRATION

                ENCODER_HALL_PHASE_CALIBRATION



        "ODrive.Encoder.Mode": {
            values
                INCREMENTAL
                HALL
                SINCOS
                SPI_ABS_CUI
                    value

                SPI_ABS_AMS
                    value

                SPI_ABS_AEAT
                    value

                SPI_ABS_RLS
                    value

                SPI_ABS_MA732
                    value



        "ODrive.Controller.ControlMode": {
            values
                VOLTAGE_CONTROL

                TORQUE_CONTROL

                VELOCITY_CONTROL

                POSITION_CONTROL



        "ODrive.Controller.InputMode": {
            values
                INACTIVE

                PASSTHROUGH

                VEL_RAMP

                POS_FILTER

                MIX_CHANNELS

                TRAP_TRAJ

                TORQUE_RAMP

                MIRROR

                TUNING



        "ODrive.Motor.MotorType": {
            values
                HIGH_CURRENT

                GIMBAL
                    value

                ACIM




}