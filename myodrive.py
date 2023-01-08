
import odrive_enums as odenums
import odrive
import asyncio
import socketio
import json
from threading import Thread
from concurrent.futures import ThreadPoolExecutor
import usb
import time

class OdriveProperty():
    def __init__(self, reference=None):
        self._reference = reference # pointer to the hardware property
    
    def _toObj(self) -> dict:
        #print("Inside _toObj()");
        return {'foo': 'bar'}

class Can(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self.error:odenums.CanError = reference.error
        self.config = {
            'baud_rate': None,
            'protocol': None
        }
        self.config = reference.config
        
        


class Endpoint(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self.endpoint = None
        self.min:float = reference.min
        self.max:float = reference.max


class Config(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        
        self.enable_uart_a:bool = reference.enable_uart_a
        self.enable_uart_b:bool = reference.enable_uart_b
        self.enable_uart_c:bool = reference.enable_uart_c
        self.uart_a_baudrate:int = reference.uart_a_baudrate
        self.uart_b_baudrate:int = reference.uart_b_baudrate
        self.uart_c_baudrate:int = reference.uart_c_baudrate
        self.enable_can_a:bool = reference.enable_can_a
        self.enable_i2c_a:bool = reference.enable_i2c_a
        self.usb_cdc_protocol:odenums.StreamProtocolType = reference.usb_cdc_protocol
        self.uart0_protocol:odenums.StreamProtocolType = reference.uart0_protocol
        self.uart1_protocol:odenums.StreamProtocolType = reference.uart1_protocol
        self.uart2_protocol:odenums.StreamProtocolType = reference.uart2_protocol
        self.max_regen_current:float = reference.max_regen_current
        self.brake_resistance:float = reference.brake_resistance
        self.enable_brake_resistor:bool = reference.enable_brake_resistor
        self.dc_bus_undervoltage_trip_level:float = reference.dc_bus_undervoltage_trip_level
        self.dc_bus_overvoltage_trip_level:float = reference.dc_bus_overvoltage_trip_level
        self.enable_dc_bus_overvoltage_ramp:bool = reference.enable_dc_bus_overvoltage_ramp
        self.dc_bus_overvoltage_ramp_start:float = reference.dc_bus_overvoltage_ramp_start
        self.dc_bus_overvoltage_ramp_end:float = reference.dc_bus_overvoltage_ramp_end
        self.dc_max_positive_current:float = reference.dc_max_positive_current
        self.dc_max_negative_current:float = reference.dc_max_negative_current
        self.error_gpio_pin:int = reference.error_gpio_pin
        self.gpio3_analog_mapping:Endpoint = Endpoint(reference.gpio3_analog_mapping)
        self.gpio4_analog_mapping:Endpoint = Endpoint(reference.gpio4_analog_mapping)



class Motor(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self.last_error_time:float = reference.last_error_time
        self.error= reference.error
        self.is_armed:bool = reference.is_armed
        self.is_calibrated:bool = reference.is_calibrated
        self.current_meas_phA:float = reference.current_meas_phA
        self.current_meas_phB:float = reference.current_meas_phB
        self.current_meas_phC:float = reference.current_meas_phC
        self.DC_calib_phA:float = reference.DC_calib_phA
        self.DC_calib_phB:float = reference.DC_calib_phB
        self.DC_calib_phC:float = reference.DC_calib_phC
        self.I_bus:float = reference.I_bus
        self.phase_current_rev_gain:float = reference.phase_current_rev_gain
        self.effective_current_lim:float = reference.effective_current_lim
        self.max_allowed_current:float = reference.max_allowed_current
        self.max_dc_calib:float = reference.max_dc_calib
        self.fet_thermistor:OnboardThermistorCurrentLimiter = OnboardThermistorCurrentLimiter(reference.fet_thermistor)
        self.motor_thermistor:OffboardThermistorCurrentLimiter = OffboardThermistorCurrentLimiter(reference.motor_thermistor)
        self.current_control = {
            'p_gain': reference.current_control.p_gain,
            'i_gain': reference.current_control.i_gain,
            'I_measured_report_filter_k': reference.current_control.I_measured_report_filter_k,
            'Id_setpoint': reference.current_control.Id_setpoint,
            'Iq_setpoint': reference.current_control.Iq_setpoint,
            'Vd_setpoint': reference.current_control.Vd_setpoint,
            'Vq_setpoint': reference.current_control.Vq_setpoint,
            'phase': reference.current_control.phase,
            'phase_vel': reference.current_control.phase_vel,
            'Ialpha_measured': reference.current_control.Ialpha_measured,
            'Ibeta_measured': reference.current_control.Ibeta_measured,
            'Id_measured': reference.current_control.Id_measured,
            'Iq_measured': reference.current_control.Iq_measured,
            'power': reference.current_control.power,
            'v_current_control_integral_d': reference.current_control.v_current_control_integral_d,
            'v_current_control_integral_q': reference.current_control.v_current_control_integral_q,
            'final_v_alpha': reference.current_control.final_v_alpha,
            'final_v_beta': reference.current_control.final_v_beta
        }
        self.n_evt_current_measurement:int = reference.n_evt_current_measurement
        self.n_evt_pwm_update:int = reference.n_evt_pwm_update
        self.config = {
            'pre_calibrated': reference.config.pre_calibrated,
            'pole_pairs': reference.config.pole_pairs,
            'calibration_current': reference.config.calibration_current,
            'resistance_calib_max_voltage': reference.config.resistance_calib_max_voltage,
            'phase_inductance': reference.config.phase_inductance,
            'phase_resistance': reference.config.phase_resistance,
            'torque_constant': reference.config.torque_constant,
            'motor_type': reference.config.motor_type,
            'current_lim': reference.config.current_lim,
            'current_lim_margin': reference.config.current_lim_margin,
            'torque_lim': reference.config.torque_lim,
            'inverter_temp_limit_lower': reference.config.inverter_temp_limit_lower,
            'inverter_temp_limit_upper': reference.config.inverter_temp_limit_upper,
            'requested_current_range': reference.config.requested_current_range,
            'current_control_bandwidth': reference.config.current_control_bandwidth,
            'acim_gain_min_flux': reference.config.acim_gain_min_flux,
            'acim_autoflux_min_Id': reference.config.acim_autoflux_min_Id,
            'acim_autoflux_enable': reference.config.acim_autoflux_enable,
            'acim_autoflux_attack_gain': reference.config.acim_autoflux_attack_gain,
            'acim_autoflux_decay_gain': reference.config.acim_autoflux_decay_gain,
            'R_wL_FF_enable': reference.config.R_wL_FF_enable,
            'bEMF_FF_enable': reference.config.bEMF_FF_enable,
            'I_bus_hard_min': reference.config.I_bus_hard_min,
            'I_bus_hard_max': reference.config.I_bus_hard_max,
            'I_leak_max': reference.config.I_leak_max,
            'dc_calib_tau': reference.config.dc_calib_tau
        }

class AcimEstimator(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self.rotor_flux = None
        self.slip_vel = None
        self.phase_offset = None
        self.stator_phase_vel = None
        self.stator_phase = None
        self.config = None
        self.slip_velocity = None


class Controller(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self.error = None
        self.last_error_time = None
        self.input_pos = None
        self.input_vel = None
        self.input_torque = None
        self.pos_setpoint = None
        self.vel_setpoint = None
        self.torque_setpoint = None
        self.trajectory_done = None
        self.vel_integrator_torque = None
        self.anticogging_valid = None
        self.autotuning_phase = None
        self.config = {
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
        self.autotuning = {
            'frequency': None,
            'pos_amplitude': None,
            'vel_amplitude': None,
            'torque_amplitude': None
        }
        self.mechanical_power = None
        self.electrical_power = None
        
    
    def move_incremental():
        pass
    
    def start_anticogging_calibration():
        pass


class Encoder(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self.error = None
        self.is_ready = None
        self.index_found = None
        self.shadow_count = None
        self.count_in_cpr  = None
        self.interpolation = None
        self.phase = None
        self.pos_estimate = None
        self.pos_estimate_counts = None
        self.pos_circular = None
        self.pos_cpr_counts = None
        self.delta_pos_cpr_counts = None
        self.hall_state = None
        self.vel_estimate = None
        self.vel_estimate_counts = None
        self.calib_scan_response = None
        self.pos_abs = None
        self.spi_error_rate = None
        self.config = {
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
                    
class SensorlessEstimator(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self.error = None
        self.phase = None
        self.pll_pos = None
        self.phase_vel = None
        self.vel_estimate = None
        self.config = {
            'observer_gain': None,
            'pll_bandwidth': None,
            'pm_flux_linkage': None
        }

class TrapezoidalTrajectory(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self.config = {
            'vel_limit':None,
            'accel_limit':None,
            'decel_limit':None
        }

class Endstop(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self.endstop_state = None
        self.config = {
            'gpio_num':None,
            'enabled':None,
            'offset':None,
            'is_active_high':None,
            'debounce_ms':None
        }
                            
class MechanicalBrake(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self.config = {
            'gpio_num':None,
            'is_active_low':None
        }
    def engage():
        pass
    
    def release():
        pass



class TaskTimer(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self.start_time = None
        self.end_time = None
        self.length = None
        self.max_length = None


class Axis(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        #self.error:odenums.AxisError = reference.error
        
        self.error:int = reference.error
        self.step_dir_active:bool = reference.step_dir_active
        self.last_drv_fault:int = reference.last_drv_fault
        self.steps:int = reference.steps
        #self.current_state:odenums.AxisState = reference.current_state
        #self.requested_state:odenums.AxisState = reference.requested_state
        self.current_state:int = reference.current_state
        self.requested_state:int = reference.requested_state
        self.is_homed:bool = reference.is_homed
        
        
        self.config:dict = {
            'startup_motor_calibration': reference.config.startup_motor_calibration,
            'startup_encoder_index_search': reference.config.startup_encoder_index_search,
            'startup_encoder_offset_calibration': reference.config.startup_encoder_offset_calibration,
            'startup_closed_loop_control': reference.config.startup_closed_loop_control,
            'startup_homing': reference.config.startup_homing,
            'enable_step_dir': reference.config.enable_step_dir,
            'step_dir_always_on': reference.config.step_dir_always_on,
            'enable_sensorless_mode': reference.config.enable_sensorless_mode,
            'watchdog_timeout': reference.config.watchdog_timeout,
            'enable_watchdog': reference.config.enable_watchdog,
            'step_gpio_pin': reference.config.step_gpio_pin,
            'dir_gpio_pin': reference.config.dir_gpio_pin,
            'calibration_lockin': {
                'current': reference.config.calibration_lockin.current,
                'ramp_time': reference.config.calibration_lockin.ramp_time,
                'ramp_distance': reference.config.calibration_lockin.ramp_distance,
                'accel': reference.config.calibration_lockin.accel,
                'vel': reference.config.calibration_lockin.vel
            },
            'sensorless_ramp': LockinConfig(reference=reference.config.sensorless_ramp),
            'general_lockin': LockinConfig(reference=reference.config.general_lockin),
            'can': CanConfig(reference=reference.config.can)
        }        

        
        self.task_times:dict = {
            'thermistor_update': TaskTimer(reference=reference.task_times.thermistor_update),
            'encoder_update': TaskTimer(reference=reference.task_times.encoder_update),
            'sensorless_estimator_update': TaskTimer(reference=reference.task_times.sensorless_estimator_update),
            'endstop_update': TaskTimer(reference=reference.task_times.endstop_update),
            'can_heartbeat': TaskTimer(reference=reference.task_times.can_heartbeat),
            'controller_update': TaskTimer(reference=reference.task_times.controller_update),
            'open_loop_controller_update': TaskTimer(reference=reference.task_times.open_loop_controller_update),
            'acim_estimator_update': TaskTimer(reference=reference.task_times.acim_estimator_update),
            'motor_update': TaskTimer(reference=reference.task_times.motor_update),
            'current_controller_update': TaskTimer(reference=reference.task_times.current_controller_update),
            'dc_calib': TaskTimer(reference=reference.task_times.dc_calib),
            'current_sense': TaskTimer(reference=reference.task_times.current_sense),
            'pwm_update': TaskTimer(reference=reference.task_times.pwm_update)
        }
     
        
        
        self.motor:Motor = Motor(reference=reference.motor)
        self.controller:Controller = Controller(reference=reference.controller)
        self.encoder:Encoder = Encoder(reference=reference.controller)
        self.acim_estimator:AcimEstimator = AcimEstimator(reference=reference.acim_estimator)
        self.sensorless_estimator:SensorlessEstimator = SensorlessEstimator(reference=reference.sensorless_estimator)
        self.trap_traj:TrapezoidalTrajectory = TrapezoidalTrajectory(reference=reference.trap_traj)
        self.min_endstop:Endstop = Endstop(reference=reference.min_endstop)
        self.max_endstop:Endstop = Endstop(reference=reference.max_endstop)
        self.mechanical_brake:MechanicalBrake = MechanicalBrake(reference=reference.mechanical_brake)
        
    
    def _toObj(self) -> dict:
        print("inside overidden _toObj in Axis")
        out = {}
        for param in dir(self):
            if param.startswith("_") == False:
                try:
                    # First, update the root properties
                    #setattr(self, param, getattr(self._odrive, param));
                    val = getattr(self, param)
                    if callable(val):
                        continue                   
                    if issubclass(type(val), OdriveProperty):
                        out[param] = val._toObj()
                    elif type(val) == dict:
                        clean_val = {}
                        for k, v in val.items():
                            if issubclass(type(v), OdriveProperty):
                                clean_val[k] = v._toObj()
                            else:
                                clean_val[k] = v
                        out[param] = clean_val
                    else:
                        out[param] = val
                
                except Exception:
                    print("Failed to get " + param)
        return out

    def watchdog_feed():
        pass



class LockinConfig(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        self.current = None
        self.ramp_time = None
        self.ramp_distance = None
        self.accel = None
        self.vel = None
        self.finish_distance = None
        self.finish_on_vel = None
        self.finish_on_distance = None
        self.finish_on_enc_idx = None


class CanConfig(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        self.node_id = None
        self.is_extended = None
        self.heartbeat_rate_ms = None
        self.encoder_rate_ms = None

class OnboardThermistorCurrentLimiter(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        self.temperature = None
        self.config = {
            'temp_limit_lower': None,
            'temp_limit_upper': None,
            'enabled': None
        }

class OffboardThermistorCurrentLimiter(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        self.temperature = None
        self.config = {
            'gpio_pin': None,
            'poly_coefficient_0': None,
            'poly_coefficient_1': None,
            'poly_coefficient_2': None,
            'poly_coefficient_3': None,
            'temp_limit_lower': None,
            'temp_limit_upper': None,
            'enabled': None
        }



class Oscilloscope(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        self.size = None
    

    def get_val():
        return {
            'in': {
                'index': None
                },
            'out': {
                'val': None
            }
        }
        
class Odrive():
    def __init__(self, serial:int=None, 
                 thread_update_time:float=1.0,
                 odrive_handle=None):
        print("Inside Odrive constructor")
        self._thread_update_time = thread_update_time
        self._odrive:odrive = None # the instance of the connected odrive
        self._json = None
        self._obj = None
        self.error:int = None
        self.vbus_voltage:float = None
        self.ibus:float = None        
        self.serial_number:int = serial
        self.ibus_report_filter_k:float = None
        self.hw_version_major:int = None
        self.hw_version_minor:int = None
        self.hw_version_variant:int = None
        self.fw_version_major:int = None
        self.fw_version_minor:int = None
        self.fw_version_revision:int = None
        self.fw_version_unreleased:int = None
        self.brake_resistor_armed:bool = None
        self.brake_resistor_saturated:bool = None
        self.brake_resistor_current:float = None
        self.n_evt_sampling:int = None
        self.n_evt_control_loop:int = None
        self.task_timers_armed:bool = None
        self.task_times = {
            'sampling': None,
            'control_loop_misc':None,
            'control_loop_checks':None,
            'dc_calib_wait':None
        }
        self.system_stats = {
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
        self.user_config_loaded:int = None
        self.misconfigured:bool = None
        self.test_property:int = None
        self.otp_valid:bool = None  

        # Now try to connect if we aren't given a valid handle
        if odrive_handle == None:
            self._odrive = odrive.find_any(serial_number=self.serial_number)
        else:
            self._odrive = odrive_handle
        print("Odrive.__init__() -- Found the odrive!")
        self.hw_version_major = getattr(self._odrive, 'hw_version_major')
        self.hw_version_minor = getattr(self._odrive, 'hw_version_minor')
        self.hw_version_variant = getattr(self._odrive, 'hw_version_variant')
        self.fw_version_major = getattr(self._odrive, 'fw_version_major')
        self.fw_version_minor = getattr(self._odrive, 'fw_version_minor')
        
        self.can:Can = Can(self._odrive.can)
        self.oscilliscope:Oscilloscope = Oscilloscope(self._odrive.oscilloscope)
        self.config:Config = Config(self._odrive.config)
        
    
    def _thread_func(self):
        #print("Inside odrive.thread_func");
        run_flag = True
        while run_flag:
            try:
                self._sync()      
                #print("Odrive[" + self.serial_number + "] vbus_voltage =  " + str(self.vbus_voltage) + " volts.")
            except Exception:
                print("Caught odrive exception in the thread function")
                run_flag = False

            time.sleep(self._thread_update_time)
    
    def _sync(self):
        print("Inside Odrive.sync() method.")
        
        out = {}
        for param in dir(self):
            if param.startswith("_") == False:
                try:
                    # First, update the root properties
                    #setattr(self, param, getattr(self._odrive, param));
                    val = getattr(self, param)
                    if callable(val):
                        continue
                    #print("Adding " + param + " (" + str(type(val)) + ") to out")
                    if issubclass(type(val), OdriveProperty):
                        out[param] = val._toObj()
                    else:
                        out[param] = val
                
                except Exception:
                    print("Failed to sync " + param)

        self._obj = out
        self._json = json.dumps(out['axis0'])
        #self._json = self._json.replace("Infinity", "NaN")
        
    def test_function(delta:int) -> float:
            pass
    
    def get_adc_voltage(gpio:int) -> float:
        pass
    
    def save_configuration() -> bool:
        pass
    
    def erase_configuration():
        pass
    
    def reboot(): 
        pass
    
    def enter_dfu_mode(): 
        pass
    
    def get_interrupt_status(irqn:int) -> int:
        pass
    
    def get_dma_status(stream_num:int) -> int:
        pass
    
    def get_gpio_states() -> int:
        pass
    
    def get_drv_fault() -> int:
        pass
    
    def clear_errors():
        pass
   
class Odrive4(Odrive):
    
    
    def __init__(self, serial:int=None,odrive_handle=None):
        print("Inside Odrive4 constructor")
        super().__init__(serial=serial, 
                         odrive_handle=odrive_handle)
        self.axis0:Axis = Axis(reference=odrive_handle.axis0)



class Odrive3(Odrive):
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
    
    
    def __init__(self, serial:int=None,odrive_handle=None):
        print("Inside Odrive3 constructor")
        super().__init__(serial=serial, 
                         odrive_handle=odrive_handle)
        self.axis0:Axis = Axis(reference=odrive_handle.axis0)
        self.axis1:Axis = Axis(reference=odrive_handle.axis1)


   
class MyOdrive():
    _threads = {}
    _odrives = {}
    _socketio:socketio.AsyncServer = None
    _broadcast_flag = False
    
    def __init__(cls):            
        cls._threads = {}
        cls._odrives = {}
        cls._socketio = None
    
    
    @classmethod
    def attachSocketIO(cls, sio:socketio.AsyncServer=None):
        if sio != None:
            sio.on("connect", cls.connect)
            cls._socketio = sio    
    
    @classmethod 
    async def connect(cls, sid, environ):
        print("Inside MyOdrive.socketio.connect()")
    
    @classmethod
    async def handleSocketMessage(cls, message):
        # print("Inside the MyOdrive SocketIO message handler")
        result = {}
        try:
            if type(message) is str:
                message = json.loads(message)
                
            if type(message) is not dict:
                #print("Failed to parse socket message in handleSocketMessage")
                result['error'] = "Invalid message format"
                return result
            
            serial_number = message.serial_number            
            drive = cls._odrives[serial_number]
            
            if drive == None:
                print("Failed to get odrive with serial number " + serial_number)
                result['error'] = "Odrive with this serial number is not found"
                return result
            else:
                result = drive._obj
        except Exception as ex:
            print("Caught exception handling the socket message in MyOdrive.")
        
        return result
    
    @classmethod
    async def list_odrives(cls):
        #print("Inside static function show_odrives()")
        try:
            return list(cls._odrives.keys())
        except:
            return []
        
    @classmethod
    def get_odrive(cls, serialnumber:str):
        #print("Inside get_odrive(" + serialnumber + ")")
        # todo return a MyOdrive instance for this specific odrive
        pass
    
    @classmethod
    def fromSerialNumber(cls, serialnumber:str):
        #print("Inside MyOdrive.fromSerialNumber()" + str)
        pass
        
    @classmethod
    async def initialize(cls):
        
        while True:
            print("MyOdrive update timer tick")
            if cls._broadcast_flag == True:
                for sn, odrv in cls._odrives.items():
                    try:
                        #await cls._socketio.emit(str(sn), odrv._json)
                        await cls._socketio.emit(str(sn), odrv._obj)
                    except Exception as ex:
                        print("Exception sending socketio message in Myodrive thread")
                
            await asyncio.sleep(5)
        
    @classmethod
    def detectUSBDevices(cls):
        while True:
            odrives = usb.core.find(idVendor=0x1209, idProduct=0x0d32, find_all=True)
            for dev in odrives:
                try:                
                    #print(dev.serial_number)
                    cls.addOdrive(dev.serial_number)
                except Exception as ex:
                    print("Exception in USB thread.")
                    print(str(ex))
            time.sleep(5)
    
    @classmethod
    def removeOdrive(cls, serial:int=None):
        print("MyOdrive.removeOdrive called for serial: " + str(serial))
        
        if serial is not None:
            cls._odrives.pop(serial)
            cls._threads.pop(serial)
        
    
    @classmethod
    def addOdrive(cls, serial:int=None):
        if serial in cls._odrives.keys():
            #print("already have this odrive: " + str(serial))
            pass
        else:
            
            odrive_handle = odrive.find_any(serial_number=serial)
            hardware_version_major = getattr(odrive_handle, "hw_version_major")
            print("Newly detected Odrive hardware version major is: " + str(hardware_version_major))
            od = None
            if hardware_version_major == 3:
                od = Odrive3(serial=serial,
                             odrive_handle=odrive_handle)
                
                
            elif hardware_version_major == 4:
                od = Odrive4(serial=serial,
                            odrive_handle=odrive_handle)
                
            else:
                od = Odrive(serial=serial,
                        odrive_handle=odrive_handle)
            
            
            cls._odrives[serial] = od
            thread = Thread(target=od._thread_func)
            thread.start()
            cls._threads[serial] = thread
            
            # Notify connected socket.io clients of a new device.
            cls._broadcast_flag = True
    

    
