
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


class Encoder():
    error = None
    is_ready = None
    index_found = None
    shadow_count = None
    count_in_cpr  = None
    interpolation = None
    phase = None
    pos_estimate = None
    pos_estimate_counts = None
    pos_circular = None
    pos_cpr_counts = None
    delta_pos_cpr_counts = None
    hall_state = None
    vel_estimate = None
    vel_estimate_counts = None
    calib_scan_response = None
    pos_abs = None
    spi_error_rate = None
    config = {
        'mode': None,
        'use_index': None,
        'index_offset': None,
        'use_index_offset': None,
        'find_idx_on_lockin_only': None,
        'abs_spi_cs_gpio_pin': None,
        'cpr': None,
        'phase_offset': None,
        'phase_offset_float': None,
        'direction': None,
        'pre_calibrated': None,
        'enable_phase_interpolation': None,
        'bandwidth': None,
        'calib_range': None,
        'calib_scan_distance': None,
        'calib_scan_omega': None,
        'ignore_illegal_hall_state': None,
        'hall_polarity': None,
        'hall_polarity_calibrated': None,
        'sincos_gpio_pin_sin': None,
        'sincos_gpio_pin_cos': None
    }

    def set_linear_count():
        pass
                    
class SensorlessEstimator():
    error = None
    phase = None
    pll_pos = None
    phase_vel = None
    vel_estimate = None
    config = {
        'observer_gain': None,
        'pll_bandwidth': None,
        'pm_flux_linkage': None
    }

class TrapezoidalTrajectory():
    config = {
        'vel_limit':None,
        'accel_limit':None,
        'decel_limit':None
    }

class Endstop():
    endstop_state = None
    config = {
        'gpio_num':None,
        'enabled':None,
        'offset':None,
        'is_active_high':None,
        'debounce_ms':None
    }
                            
class MechanicalBrake():
    config = {
        'gpio_num':None,
        'is_active_low':None
    }
    def engage():
        pass
    
    def release():
        pass



class TaskTimer():
    start_time = None
    end_time = None
    length = None
    max_length = None

class ODrive3():
    config = {
        'gpio1_mode': None,
        'gpio2_mode': None,
        'gpio3_mode': None,
        'gpio4_mode': None,
        'gpio5_mode': None,
        'gpio6_mode': None,
        'gpio7_mode': None,
        'gpio8_mode': None,
        'gpio9_mode': None,
        'gpio10_mode': None,
        'gpio11_mode': None,
        'gpio12_mode': None,
        'gpio13_mode': None,
        'gpio14_mode': None,
        'gpio15_mode': None,
        'gpio16_mode': None,
        'gpio1_pwm_mapping': None,
        'gpio2_pwm_mapping': None,
        'gpio3_pwm_mapping': None,
        'gpio4_pwm_mapping': None,
    }
    axis0 = None
    axis1 = None
