
import odrive_enums as odenums
import odrive
import asyncio
import socketio
import json
from threading import Thread
from concurrent.futures import ThreadPoolExecutor
import usb
import time
import math
import serial
import numpy

class OdriveProperty():
    def __init__(self, reference=None):
        self._reference = reference # pointer to the hardware property
 
    def _toObj(self) -> dict:
        out = { 'class_name': type(self).__name__ }
        for param in dir(self):
            if param.startswith("_") == True:
                continue
            else:
                try:
                    # First, update the root properties
                    #setattr(self, param, getattr(self._odrive, param));
                    val = getattr(self, param)
                    if callable(val):
                        continue                   
                    if issubclass(type(val), OdriveProperty):
                        out[param] = val._toObj()
                    elif type(val) == dict:
                        out[param] = val = self._dictToObj(val)
                    else:
                        out[param] = self._replaceInfNan(val)
                
                except Exception as e:
                    print("Failed to get _toObj base class: " + str(type(self)) + "->" + param)
                    print(e)
        return out
    
    def _update(self):
        # Update all of the class properties with the reference pointer
        # values. Override this in classes that use this as a base.
        pass
    
    # Recursively convert dictionary contents to objects to make
    # them json safe
    def _dictToObj(self, thedict:dict=None) -> dict:
        if thedict is None:
            raise Exception("Invalid input")
    
        out = {}
        for key, val in thedict.items():
            newval = val
            if callable(val):
                continue
            elif type(val) is dict:
                newval = self._dictToObj(val)
            elif issubclass(type(val), OdriveProperty):
                newval = val._toObj()
            else:
                newval = self._replaceInfNan(val)
            out[key] = newval
        
        return out
    
    def _replaceInfNan(self, value):
        try:
            if type(value) != float:
                return value
            if math.isinf(value):
                if value > 0:
                    #value = "Infinity"
                    return "foobarInfinity"
                else:
                    #value = "-Infinity"
                    return "foobar-Infinity"
            if str(value) == "nan":
                #return "NaN"
                return "foobarNaN"
        except Exception as ex:
            print("Failed checking for bogus number: " + str(value))
            print(ex)
    
        return value

class Can(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        # Odd, one axis is different than the other.
        #self.error:int = reference.error
        #self.error = "foo"
        #self.config = {
        #    'baud_rate': reference.config.baud_rate,
        #    'protocol': reference.config.protocol
        #}
        self._update()
        
    def _update(self):
        self.error = "Not Implemented in MyOdrive"
        
        


class Endpoint(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.endpoint = 0
        self.min:float = reference.min
        self.max:float = reference.max


class Config(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        self._update()    

    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
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
        #self.gpio3_analog_mapping:Endpoint = Endpoint(reference.gpio3_analog_mapping)
        #self.gpio4_analog_mapping:Endpoint = Endpoint(reference.gpio4_analog_mapping)



class Motor(OdriveProperty):
    class MotorConfig(OdriveProperty):
        def __init__(self, reference=None):
            super().__init__(reference)
            self._update()
            
        def _update(self):
            if self._reference is None:
                return
            reference = self._reference
            self.pre_calibrated = reference.pre_calibrated
            self.pole_pairs = reference.pole_pairs
            self.calibration_current= reference.calibration_current
            self.resistance_calib_max_voltage = reference.resistance_calib_max_voltage
            self.phase_inductance = reference.phase_inductance
            self.phase_resistance = reference.phase_resistance
            self.torque_constant = reference.torque_constant
            self.motor_type = reference.motor_type
            self.current_lim = reference.current_lim
            self.current_lim_margin = reference.current_lim_margin
            self.torque_lim = reference.torque_lim
            self.inverter_temp_limit_lower = reference.inverter_temp_limit_lower
            self.inverter_temp_limit_upper= reference.inverter_temp_limit_upper
            self.requested_current_range = reference.requested_current_range
            self.current_control_bandwidth = reference.current_control_bandwidth
            self.acim_gain_min_flux = reference.acim_gain_min_flux
            self.acim_autoflux_min_Id =  reference.acim_autoflux_min_Id
            self.acim_autoflux_enable = reference.acim_autoflux_enable
            self.acim_autoflux_attack_gain = reference.acim_autoflux_attack_gain
            self.acim_autoflux_decay_gain = reference.acim_autoflux_decay_gain
            self.R_wL_FF_enable = reference.R_wL_FF_enable
            self.bEMF_FF_enable = reference.bEMF_FF_enable
            self.I_bus_hard_min = reference.I_bus_hard_min
            self.I_bus_hard_max = reference.I_bus_hard_max
            self.I_leak_max = reference.I_leak_max
            self.dc_calib_tau = reference.dc_calib_tau
    
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
    
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
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
        self.config:self.MotorConfig = self.MotorConfig(reference.config)
        '''
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
        '''

class AcimEstimator(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.rotor_flux:float = reference.rotor_flux
        self.slip_vel:float = reference.slip_vel
        self.phase_offset:float = reference.phase_offset
        self.stator_phase_vel:float = reference.stator_phase_vel
        self.stator_phase:float = reference.stator_phase
        self.config = {
            'slip_velocity': reference.config.slip_velocity
        }
    

class Controller(OdriveProperty):
    class ControlMode(OdriveProperty):
        def __init__(self, reference=None):
            super().__init__(reference=reference)
            self._update()
        
        def _update(self):
            if self._reference is None:
                return
            reference = self._reference
            self.values = {
                'VOLTAGE_CONTROL': { 'doc': 'foo'},
                'TORQUE_CONTROL': { 'doc': 'foo'},
                'VELOCITY_CONTROL':  { 'doc': 'foo'},
                'POSITION_CONTROL':  { 'doc': 'foo'}
            }
            
    class Config(OdriveProperty):
        def __init__(self, reference=None):
            super().__init__(reference=reference)
            self._update()
            
        def _update(self):
            if self._reference is None:
                return
            reference = self._reference

            self.gain_scheduling_width = reference.gain_scheduling_width
            self.enable_vel_limit = reference.enable_vel_limit
            self.enable_torque_mode_vel_limit = reference.enable_torque_mode_vel_limit
            self.enable_gain_scheduling = reference.enable_gain_scheduling
            self.enable_overspeed_error = reference.enable_overspeed_error
            self.control_mode = reference.control_mode
            #'control_mode': self.ControlMode(reference.config.control_mode),
            self.input_mode = reference.input_mode
            #'input_mode': self.InputMode(reference.config.input_mode),
            self.pos_gain = reference.pos_gain
            self.vel_gain = reference.vel_gain
            self.vel_integrator_gain = reference.vel_integrator_gain
            self.vel_integrator_limit = reference.vel_integrator_limit
            self.vel_limit = reference.vel_limit
            self.vel_limit_tolerance = reference.vel_limit_tolerance
            self.vel_ramp_rate = reference.vel_ramp_rate
            self.torque_ramp_rate = reference.torque_ramp_rate
            self.circular_setpoints = reference.circular_setpoints
            self.circular_setpoint_range = reference.circular_setpoint_range
            self.steps_per_circular_range = reference.steps_per_circular_range
            self.homing_speed = reference.homing_speed
            self.inertia = reference.inertia
            self.axis_to_mirror = reference.axis_to_mirror
            self.mirror_ratio = reference.mirror_ratio
            self.torque_mirror_ratio = reference.torque_mirror_ratio
            self.load_encoder_axis = reference.load_encoder_axis
            self.input_filter_bandwidth = reference.input_filter_bandwidth
            self.anticogging = {
                'index': reference.anticogging.index,
                'pre_calibrated': reference.anticogging.pre_calibrated,
                'calib_anticogging': reference.anticogging.calib_anticogging,
                'calib_pos_threshold': reference.anticogging.calib_pos_threshold,
                'calib_vel_threshold': reference.anticogging.calib_vel_threshold,
                'cogging_ratio': reference.anticogging.cogging_ratio,
                'anticogging_enabled': reference.anticogging.anticogging_enabled
            },
            self.mechanical_power_bandwidth = reference.mechanical_power_bandwidth
            self.electrical_power_bandwidth = reference.electrical_power_bandwidth
            self.spinout_mechanical_power_threshold = reference.spinout_mechanical_power_threshold
            self.spinout_electrical_power_threshold = reference.spinout_electrical_power_threshold
            
    
    class InputMode(OdriveProperty):
        def __init__(self, reference=None):
            super().__init__(reference=reference)
            self._update()
        
        def _update(self):
            if self._reference is None:
                return
            reference = self._reference
            self.values = {
                'INACTIVE': { 'doc': 'foo', 'brief': 'bar'},
                'PASSTHROUGH': { 'doc': 'foo', 'brief': 'bar'},
                'VEL_RAMP':  { 'doc': 'foo', 'brief': 'bar'},
                'POS_FILTER':  { 'doc': 'foo', 'brief': 'bar'},
                'MIX_CHANNELS':  { 'doc': 'foo', 'brief': 'bar'},
                'TRAP_TRAJ':  { 'doc': 'foo', 'brief': 'bar'},
                'TORQUE_RAMP':  { 'doc': 'foo', 'brief': 'bar'},
                'MIRROR':  { 'doc': 'foo', 'brief': 'bar'},
                'TUNING':  { 'doc': 'foo', 'brief': 'bar'}
            }
            
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.error:int = reference.error
        self.last_error_time:float = reference.last_error_time
        self.input_pos:float = reference.input_pos
        self.input_vel:float = reference.input_vel
        self.input_torque:float = reference.input_torque
        self.pos_setpoint:float = reference.pos_setpoint
        self.vel_setpoint:float = reference.vel_setpoint
        self.torque_setpoint:float = reference.torque_setpoint
        self.trajectory_done:bool = reference.trajectory_done
        self.vel_integrator_torque:float = reference.vel_integrator_torque
        self.anticogging_valid:float = reference.anticogging_valid
        self.autotuning_phase:float = reference.autotuning_phase
        self.config = reference.config
        """
        
        """
        self.autotuning = {
            'frequency': reference.autotuning.frequency,
            'pos_amplitude': reference.autotuning.pos_amplitude,
            'vel_amplitude': reference.autotuning.vel_amplitude,
            'torque_amplitude': reference.autotuning.torque_amplitude
        }
        self.mechanical_power:float = reference.mechanical_power
        self.electrical_power:float = reference.electrical_power
        self.config:self.Config = self.Config(reference=reference.config)
        

class Encoder(OdriveProperty):
    class Mode(OdriveProperty):
        def __init__(self, reference=None):
            super().__init__(self, reference)
            self._update()
            
        def _update(self):
            if self._reference is None:
                return
            reference = self._reference
            self.values = {
                'INCREMENTAL': None,
                'HALL': None,
                'SINCOS': None,
                'SPI_ABS_CUI': 256,
                'SPI_ABS_AMS': 257,
                'SPI_ABS_AEAT': 258,
                'SPI_ABS_RLS': 259,
                'SPI_ABS_MA732':260
            }
        
        
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.error:int = reference.error
        self.is_ready:bool = reference.is_ready
        self.index_found:bool = reference.index_found
        self.shadow_count:int = reference.shadow_count
        self.count_in_cpr:int  = reference.count_in_cpr
        self.interpolation:float = reference.interpolation
        self.phase:float = reference.phase
        self.pos_estimate:float = reference.pos_estimate
        self.pos_estimate_counts:float = reference.pos_estimate_counts
        self.pos_circular:float = reference.pos_circular
        self.pos_cpr_counts:float = reference.pos_cpr_counts
        self.delta_pos_cpr_counts:float = reference.delta_pos_cpr_counts
        self.hall_state:int = reference.hall_state
        self.vel_estimate:float = reference.vel_estimate
        self.vel_estimate_counts:float = reference.vel_estimate_counts
        self.calib_scan_response:float = reference.calib_scan_response
        self.pos_abs:int = reference.pos_abs
        self.spi_error_rate:float = reference.spi_error_rate
    
        self.config = {
            'mode': reference.config.mode,
            'use_index': reference.config.use_index,
            'index_offset': reference.config.index_offset,
            'use_index_offset': reference.config.use_index_offset,
            'find_idx_on_lockin_only': reference.config.find_idx_on_lockin_only,
            'abs_spi_cs_gpio_pin': reference.config.abs_spi_cs_gpio_pin,
            'cpr': reference.config.cpr,
            'phase_offset': reference.config.phase_offset,
            'phase_offset_float': reference.config.phase_offset_float,
            'direction': reference.config.direction,
            'pre_calibrated': reference.config.pre_calibrated,
            'enable_phase_interpolation': reference.config.enable_phase_interpolation,
            'bandwidth': reference.config.bandwidth,
            'calib_range': reference.config.calib_range,
            'calib_scan_distance': reference.config.calib_scan_distance,
            'calib_scan_omega': reference.config.calib_scan_omega,
            'ignore_illegal_hall_state': reference.config.ignore_illegal_hall_state,
            'hall_polarity': reference.config.hall_polarity,
            'hall_polarity_calibrated': reference.config.hall_polarity_calibrated,
            'sincos_gpio_pin_sin': reference.config.sincos_gpio_pin_sin,
            'sincos_gpio_pin_cos': reference.config.sincos_gpio_pin_cos
        }
        

    def set_linear_count():
        pass
                    
class SensorlessEstimator(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
    
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.error = reference.error
        self.phase = reference.phase
        self.pll_pos = reference.pll_pos
        self.phase_vel = reference.phase_vel
        self.vel_estimate = reference.vel_estimate
        self.config = {
            'observer_gain': reference.config.observer_gain,
            'pll_bandwidth': reference.config.pll_bandwidth,
            'pm_flux_linkage': reference.config.pm_flux_linkage
        }
        
    

class TrapezoidalTrajectory(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.config = {
            'vel_limit': reference.config.vel_limit,
            'accel_limit': reference.config.accel_limit,
            'decel_limit': reference.config.decel_limit
        }
        

class Endstop(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
    
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.endstop_state = reference.endstop_state
        self.config = {
            'gpio_num': reference.config.gpio_num,
            'enabled':reference.config.enabled,
            'offset':reference.config.offset,
            'is_active_high':reference.config.is_active_high,
            'debounce_ms':reference.config.debounce_ms
        }
    
                            
class MechanicalBrake(OdriveProperty):
    class Config(OdriveProperty):
        def __init__(self, reference=None):
            super().__init__(reference)
            self._update()
        
        def _update(self):
            if self._reference is None:
                return
            reference = self._reference
            self.gpio_num = reference.gpio_num
            self.is_active_low = reference.is_active_low
        
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.config:self.Config = self.Config(reference=reference.config)
        
    def engage():
        pass
    
    def release():
        pass

    



class TaskTimer(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.start_time = reference.start_time
        self.end_time = reference.end_time
        self.length = reference.length
        self.max_length = reference.max_length
        
    


class Axis(OdriveProperty):
    class Config(OdriveProperty):
        def __init__(self, reference=None):
            super().__init__(reference)
            self._update()
            
        def _update(self):
            if self._reference is None:
                return
            reference = self._reference
            self.startup_motor_calibration = reference.startup_motor_calibration
            self.startup_encoder_index_search = reference.startup_encoder_index_search
            self.startup_encoder_offset_calibration = reference.startup_encoder_offset_calibration
            self.startup_closed_loop_control = reference.startup_closed_loop_control
            self.startup_homing = reference.startup_homing
            self.enable_step_dir = reference.enable_step_dir
            self.step_dir_always_on = reference.step_dir_always_on
            self.enable_sensorless_mode = reference.enable_sensorless_mode
            self.watchdog_timeout = reference.watchdog_timeout
            self.enable_watchdog = reference.enable_watchdog
            self.step_gpio_pin = reference.step_gpio_pin
            self.dir_gpio_pin = reference.dir_gpio_pin
            self.calibration_lockin = CalibrationLockinConfig(reference=reference.calibration_lockin)    
            self.sensorless_ramp = LockinConfig(reference=reference.sensorless_ramp)
            self.general_lockin = LockinConfig(reference=reference.general_lockin)
            self.can = Can(reference=reference.can)
   
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.error:int = reference.error        
        self.error:int = reference.error
        self.step_dir_active:bool = reference.step_dir_active
        self.last_drv_fault:int = reference.last_drv_fault
        self.steps:int = reference.steps
        self.current_state:int = reference.current_state
        self.requested_state:int = reference.requested_state
        self.current_state:int = reference.current_state
        self.requested_state:int = reference.requested_state
        self.is_homed:bool = reference.is_homed
        
        
        self.config:self.Config = self.Config(reference=reference.config)

        
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
        self.encoder:Encoder = Encoder(reference=reference.encoder)
        self.acim_estimator:AcimEstimator = AcimEstimator(reference=reference.acim_estimator)
        self.sensorless_estimator:SensorlessEstimator = SensorlessEstimator(reference=reference.sensorless_estimator)
        self.trap_traj:TrapezoidalTrajectory = TrapezoidalTrajectory(reference=reference.trap_traj)
        self.min_endstop:Endstop = Endstop(reference=reference.min_endstop)
        self.max_endstop:Endstop = Endstop(reference=reference.max_endstop)
        self.mechanical_brake:MechanicalBrake = MechanicalBrake(reference=reference.mechanical_brake)
    
    def watchdog_feed():
        pass



class LockinConfig(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.current = reference.current
        self.ramp_time = reference.ramp_time
        self.ramp_distance = reference.ramp_distance
        self.accel = reference.accel
        self.vel = reference.vel
        self.finish_distance = reference.finish_distance
        self.finish_on_vel = reference.finish_on_vel
        self.finish_on_distance = reference.finish_on_distance
        self.finish_on_enc_idx = reference.finish_on_enc_idx


class CalibrationLockinConfig(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference)
        self._update()
    
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.current = reference.current
        self.ramp_time = reference.ramp_time
        self.ramp_distance =reference.ramp_distance
        self.accel = reference.accel
        self.vel = reference.vel   

class CanConfig(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.node_id = reference.node_id
        self.is_extended = reference.is_extended
        self.heartbeat_rate_ms = reference.heartbeat_rate_ms
        self.encoder_rate_ms = reference.encoder_rate_ms
    
    
    

class OnboardThermistorCurrentLimiter(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.temperature = reference.temperature
        self.config = {
            'temp_limit_lower': reference.config.temp_limit_lower,
            'temp_limit_upper': reference.config.temp_limit_upper,
            'enabled': reference.config.enabled
        }
    

class OffboardThermistorCurrentLimiter(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        self._update()
        
    def _update(self):
        if self._reference is None:
            return
        reference = self._reference
        self.temperature = reference.temperature
        self.config = {
            'gpio_pin': reference.config.gpio_pin,
            'poly_coefficient_0': reference.config.poly_coefficient_0,
            'poly_coefficient_1': reference.config.poly_coefficient_1,
            'poly_coefficient_2': reference.config.poly_coefficient_2,
            'poly_coefficient_3': reference.config.poly_coefficient_3,
            'temp_limit_lower': reference.config.temp_limit_lower,
            'temp_limit_upper': reference.config.temp_limit_upper,
            'enabled': reference.config.enabled
        }




class Oscilloscope(OdriveProperty):
    def __init__(self, reference=None):
        super().__init__(reference=reference);
        self._update()
    
    def _update(self):
        if self._reference is None:
            return
        
        self.size = self._reference.size

    def get_val():
        return {
            'in': {
                'index': None
                },
            'out': {
                'val': None
            }
        }
        
    
        
class Odrive(OdriveProperty):
    def __init__(self, serial_number=None,
                 thread_update_time:float=1.0,
                 odrive_handle=None):
        print("Inside Odrive constructor")
        # Now try to connect if we aren't given a valid handle
        if serial_number is None:
            print("Error creating Odrive without serial number")
            raise Exception("Error creating odrive")
        self.serial_number = serial_number
        if odrive_handle == None:
            self._odrive = odrive.find_any(serial_number=self.serial_number)
            print("Odrive.__init__() -- Found the odrive!")
        else:
            self._odrive = odrive_handle
            print("Odrive.__init__() -- passed in the odrive!")
        self._is_telemetry_running = False
        self._thread_update_time = thread_update_time 
        self._telem_update_time = thread_update_time / 5       
        self._json = None
        self._obj = None
        self._telem_properties = {}
        self._telemObj = None
        self._reboot = False
        self._telemThreadLocked = False
        self._reference = self._odrive # at some point I need to fix this.
        self._update()

    async def onTelemetry(self):
        print(f"inside onTelemetry")
        
    async def addTelemetry(self, properties):
        print(f"Adding telemetry to odrive {self.serial_number}")
        try:
            print(properties)
            for new_property in properties:                
                self._telem_properties[new_property] = await self._findHandleFromString(new_property)
        except Exception as ex:
            print(ex)
   
    async def _findHandleFromString(self, tree_string):
        tree = tree_string.split(".")
        new_property = tree.pop()
        property = self
        t = ""
        for t in tree:
            print(t)
            p = getattr(property, t)
            if issubclass(type(p), OdriveProperty):
                property = p
            
        return { new_property: property._reference }
        
    async def removeTelemetry(self, properties):
        print(f"Removing telemetry properties from odrive {self.serial_number}")
        print(properties)
        try:
            for del_property in properties:
                self._telem_properties.pop(del_property)
        except Exception as ex:
            print(ex)
    
    # A higher resolution version of _update for a specific subset of properties
    def _telem(self):
        if self._reboot:
            return
     
        # todo fill out the data for this.
        to = {}
        to['timestamp'] = time.time()
        if self._odrive is None:
            self._telemThreadLocked = False
            return
        
        self._telemThreadLocked = True
        try:
            for property in self._telem_properties:
                val = self._telem_properties[property]
                for key in val:
                    to[property] = getattr(val[key], key)
                
            self._telemObj = to
        except Exception as e:
            print("Error in Odrive._telem() {}".format(e))
        finally:
            self._telemThreadLocked = False    
        
        
    def _update(self):
        if self._reboot:
            return
        odrive_handle = self._odrive
        if odrive_handle == None:
            return
        # update the properies.
        print("Inside Odrive._update()")
        self.error:int = odrive_handle.error
        self.vbus_voltage:float = odrive_handle.vbus_voltage
        self.ibus:float = odrive_handle.ibus
        
        self.ibus_report_filter_k:float = odrive_handle.ibus_report_filter_k
        self.hw_version_major:int = odrive_handle.hw_version_major
        self.hw_version_minor:int = odrive_handle.hw_version_minor
        self.hw_version_variant:int = odrive_handle.hw_version_variant
        self.fw_version_major:int = odrive_handle.fw_version_major
        self.fw_version_minor:int = odrive_handle.fw_version_minor
        self.fw_version_revision:int = odrive_handle.fw_version_revision
        self.fw_version_unreleased:int = odrive_handle.fw_version_unreleased
        
        self.n_evt_sampling:int = odrive_handle.n_evt_sampling
        self.n_evt_control_loop:int = odrive_handle.n_evt_control_loop
        self.task_timers_armed:bool = odrive_handle.task_timers_armed
        self.task_times = {
            'sampling': TaskTimer(odrive_handle.task_times.sampling),
            'control_loop_misc': TaskTimer(odrive_handle.task_times.control_loop_misc),
            'control_loop_checks':TaskTimer(odrive_handle.task_times.control_loop_checks),
            'dc_calib_wait': TaskTimer(odrive_handle.task_times.dc_calib_wait)
        }
        self.system_stats = {
            'uptime': odrive_handle.system_stats.uptime,
            'min_heap_space': odrive_handle.system_stats.min_heap_space,
            'max_stack_usage_axis': odrive_handle.system_stats.max_stack_usage_axis,
            'max_stack_usage_usb': odrive_handle.system_stats.max_stack_usage_usb,
            'max_stack_usage_uart': odrive_handle.system_stats.max_stack_usage_uart,
            'max_stack_usage_can': odrive_handle.system_stats.max_stack_usage_can,
            'max_stack_usage_startup': odrive_handle.system_stats.max_stack_usage_startup,
            'max_stack_usage_analog': odrive_handle.system_stats.max_stack_usage_analog,
            'stack_size_axis': odrive_handle.system_stats.stack_size_axis,
            'stack_size_usb': odrive_handle.system_stats.stack_size_usb,
            'stack_size_uart': odrive_handle.system_stats.stack_size_uart,
            'stack_size_startup': odrive_handle.system_stats.stack_size_startup,
            'stack_size_can': odrive_handle.system_stats.stack_size_can,
            'stack_size_analog': odrive_handle.system_stats.stack_size_analog,
            'prio_axis': odrive_handle.system_stats.prio_axis,
            'prio_usb': odrive_handle.system_stats.prio_usb,
            'prio_uart': odrive_handle.system_stats.prio_uart,
            'prio_startup': odrive_handle.system_stats.prio_startup,
            'prio_can': odrive_handle.system_stats.prio_can,
            'prio_analog': odrive_handle.system_stats.prio_analog,
            'usb': {                                                
                    'rx_cnt': odrive_handle.system_stats.usb.rx_cnt,
                    'tx_cnt': odrive_handle.system_stats.usb.tx_cnt,
                    'tx_overrun_cnt': odrive_handle.system_stats.usb.tx_overrun_cnt
            },
            'i2c': {                                                
                    'addr': odrive_handle.system_stats.i2c.addr,
                    'addr_match_cnt': odrive_handle.system_stats.i2c.addr_match_cnt,
                    'rx_cnt': odrive_handle.system_stats.i2c.rx_cnt,
                    'error_cnt': odrive_handle.system_stats.i2c.error_cnt,
            }
        }
        self.user_config_loaded:int = odrive_handle.user_config_loaded
        self.misconfigured:bool = odrive_handle.misconfigured
        self.test_property:int = odrive_handle.test_property
        self.otp_valid:bool = odrive_handle.otp_valid

        
       
        
        self.can:Can = Can(odrive_handle.can)
        self.oscilliscope:Oscilloscope = Oscilloscope(odrive_handle.oscilloscope)
        self.config:Config = Config(odrive_handle.config)
    
    def _thread_func(self):
        #print("Inside odrive.thread_func");
        count = 0
        while True:
            if self._reboot == False:
                try:
                    
                    if MyOdrive.joystick['enabled'] is True:
                        #print(f"joystick commanding input_pos {MyOdrive.joystick['x']} and {MyOdrive.joystick['y']}")
                        self.axis0.controller._reference.input_pos = MyOdrive.joystick['y']
                        self.axis1.controller._reference.input_pos = MyOdrive.joystick['x']
                    
                    self._telem()
                    """
                    if count > 100:
                        self._sync()
                        count = 0                    
                    count = count + 1
                    """
                    # TODO this needs to be faster.
                    #print("Odrive[" + self.serial_number + "] vbus_voltage =  " + str(self.vbus_voltage) + " volts.")
                except Exception as ex:
                    print("Caught odrive exception in the thread function")
                    print(ex)
                    #run_flag = False

            time.sleep(0.05)
        
        # reset the reboot flag here
    
    def _sync(self):
        print("Inside Odrive.sync() method.")
        self._update()
        self._obj = self._toObj() # This fixes Infinity/NaN issues
        # add the telemetry data
        self._obj['telemetry'] = list(self._telem_properties.keys())
        try:
            self._json = json.dumps(self._obj, indent=True)
        except Exception as ex:
            print(ex)
        
        #self._json = self._json.replace("Infinity", "NaN")
        return self._obj
        
    async def _telemetryTask(self):
        print("Inside Odrive._telemetryTask") 
        while True:
            while self._telemThreadLocked:
                await asyncio.sleep(0.1)
            await MyOdrive.onTelemetry(self.serial_number, self._telemObj)
            #print("Odrive._telemetryTask tick.")
            await asyncio.sleep(self._telem_update_time)
            if self._is_telemetry_running is False:
                return
        print("Odrive._telemetryTask exiting.")
        
    def test_function(self, delta:int) -> float:
        print("Inside Odrive.test_function")
        return self._odrive.test_function(delta)
        
    
    def get_adc_voltage(self, gpio:int) -> float:
        return self._odrive.get_adc_voltage(gpio)

    
    def save_configuration(self) -> bool:
        # Let's try the same reboot flag.
        self._reboot = True
        time.sleep(1)
        return self._odrive.save_configuration()

    
    def erase_configuration(self):
        return self._odrive.erase_configuration()

    
    def reboot(self): 
        # we need to stop the running thread before rebooting.
        self._reboot = True
        time.sleep(1)
        return self._odrive.reboot()

    
    def enter_dfu_mode(self): 
        return self._odrive.enter_dfu_mode()

    
    def get_interrupt_status(self, irqn:int) -> int:
        return self._odrive.get_interrupt_status(irqn)

    
    def get_dma_status(self, stream_num:int) -> int:
        return self._odrive.get_dma_status(stream_num)

    
    def get_gpio_states(self) -> int:
        return self._odrive.get_gpio_states()

    
    def get_drv_fault(self) -> int:
        return self._odrive.get_drv_fault()

    
    def clear_errors(self):
        return self._odrive.clear_errors()
        
   
class Odrive4(Odrive):
    
    
    def __init__(self, serial_number=None,odrive_handle=None):
        print("Inside Odrive4 constructor")
        super().__init__(serial_number=serial_number, 
                         odrive_handle=odrive_handle)
        self._odrive = odrive_handle
        self._update()
    
    def _update(self):
        super()._update()
        if self._odrive is None:
            return
        print("Inside Odrive4._update()")
        odrive_handle = self._odrive
        self.axis0:Axis = Axis(reference=odrive_handle.axis0)
        



class Odrive3(Odrive):
    config = {
        'gpio1_mode': 0,
        'gpio2_mode': 0,
        'gpio3_mode': 0,
        'gpio4_mode': 0,
        'gpio5_mode': 0,
        'gpio6_mode': 0,
        'gpio7_mode': 0,
        'gpio8_mode': 0,
        'gpio9_mode': 0,
        'gpio10_mode': 0,
        'gpio11_mode': 0,
        'gpio12_mode': 0,
        'gpio13_mode': 0,
        'gpio14_mode': 0,
        'gpio15_mode': 0,
        'gpio16_mode': 0,
        'gpio1_pwm_mapping': 0,
        'gpio2_pwm_mapping': 0,
        'gpio3_pwm_mapping': 0,
        'gpio4_pwm_mapping': 0,
    }
    
    
    def __init__(self, serial_number=None,odrive_handle=None):
        print("Inside Odrive3 constructor")
        super().__init__(serial_number=serial_number, odrive_handle=odrive_handle)
        self._odrive = odrive_handle
        self.serial_number = serial_number
        self._update()                 
        
    def _update(self):
        super()._update()
        if self._odrive is None:
            return

        print("Inside Odrive3._update()")
        odrive_handle = self._odrive   
        self.axis0:Axis = Axis(reference=odrive_handle.axis0)
        self.axis1:Axis = Axis(reference=odrive_handle.axis1)     
        self.brake_resistor_armed:bool = odrive_handle.brake_resistor_armed
        self.brake_resistor_saturated:bool = odrive_handle.brake_resistor_saturated
        self.brake_resistor_current:float = odrive_handle.brake_resistor_current


   
class MyOdrive():
    _threads = {}
    _odrives = {}
    _socketio:socketio.AsyncServer = None
    _broadcast_flag = False
    _newodrives = []
    _telemetry_tasks = {}
    _remoteFunctionRequests = []
    _remotePropertyRequests = []
    _socketSendQueue = []
    _joystickThread = None
    joystick = { 'x': None, 
                'y': None, 
                'x_median': None, 
                'y_median': None,
                'enabled': False,
                }

    
    def __init__(cls):        
        print("Inside MyOdrive constructor")
        cls._threads = {}
        cls._odrives = {}
        cls._socketio = None
    
    @classmethod
    def initializeThreads(cls):
        cls._usbThread = Thread(target=cls.detectUSBDevices)
        cls._usbThread.start()
        cls._remoteQueueThread = Thread(target=cls.processRemoteRequests)
        cls._remoteQueueThread.start()
        cls._joystickThread = Thread(target=cls.joystickThreadFunc)
        cls._joystickThread.start()
        
    @classmethod
    def joystickThreadFunc(cls):
        print("Starting the joystick thread")
        ser = serial.Serial('COM12', 9600, timeout=1)
        time.sleep(2)
        while True:
            line = ser.readline()
            if line:
                line_str = line.decode().strip()
                
                split = line_str.split(",")
                if len(split) == 2:
                    x = (float(split[0]) - 512.0) / 1024.0
                    y = (float(split[1]) - 512.0) / 1024.0 
                    
                    cls.joystick['timestamp'] = time.time()
                    
                    if cls.joystick['x_median'] is None:
                        # initialize it.
                        cls.joystick['x'] = x
                        cls.joystick['y'] = y
                        cls.joystick['x_median'] = [cls.joystick['x']] * 5
                        cls.joystick['y_median'] = [cls.joystick['y']] * 5
                    else :
                        cls.joystick['x_median'].insert(0, x)
                        cls.joystick['y_median'].insert(0, y)
                        cls.joystick['x_median'].pop()
                        cls.joystick['y_median'].pop()
                        x = numpy.median(cls.joystick['x_median'])
                        y = numpy.median(cls.joystick['y_median'])
                    
                    cls.joystick['x'] = x
                    cls.joystick['y'] = y
                    
                                                                                
                #print(cls.joystick)
            
        
    @classmethod
    async def asyncUpdate(cls):
        while True:
            #print("MyOdrive update timer tick")
            # If a client connects before an odrive is detected,
            # they wont know about it. We will broadcast a list of all
            # connected odrives whenever an odrive connects or disconnects
            if cls._broadcast_flag == True:
                for sn, odrv in cls._odrives.items():
                    try:
                        #await cls._socketio.emit(str(sn), odrv._json)
                        #await cls._socketio.emit(str(sn), odrv._obj)
                        await cls._socketio.emit("list_odrives", list(cls._odrives.keys()))
                    except Exception as ex:
                        print("Exception sending socketio message in Myodrive thread")
                        print(ex)
                        
            cls._broadcast_flag = False
            
            # process any socket send requests in _socketSendQueue
            while len(cls._socketSendQueue) > 0:
                req = cls._socketSendQueue.pop(0)
                try:
                    print("Sending socketio message in Myodrive thread: " + req['request_id'] + " " + str(req['result']))
                    await cls._socketio.emit(req['request_id'], { 'result': req['result'] })
                except Exception as ex:
                    print("Exception sending socketio message in Myodrive thread")
                    print(str(ex))
            
            #await cls._socketio.emit("joystick", cls.joystick)
            await asyncio.sleep(0.3)

    @classmethod
    def sendSocketMessageSync(cls, request_id, result):
        cls._socketSendQueue.append({'request_id': request_id, 'result': result})
        
    
    @classmethod
    def attachSocketIO(cls, sio:socketio.AsyncServer=None):
        print("MyOdrive.attachSocketIO")
        if sio != None:
            sio.on("connect", cls.connect)
            sio.on("telem_start", cls.startTelemetry)
            sio.on("telem_stop", cls.stopTelemetry)
            sio.on("sync", cls.syncOdrive)
            sio.on("call", cls.remoteCall) # a function for an odrive being called remotely
            sio.on("set", cls.remotePropertySet) # a property is being set remotely
            sio.on("add_telemetry", cls.addTelemetry)
            sio.on("remove_telemetry", cls.removeTelemetry)
            sio.on("joystick", cls.joystickRemoteEvent)
            cls._socketio = sio    
            for od in cls._odrives.values():
                od._socketio = sio # pass a handle to sio to the od instance

            print("MyDrive.attachSocketIO() finished")
        else:
            print("Error, sio is None")
    
    @classmethod
    async def joystickRemoteEvent(cls, sid, message):
        print("Inside MyOdrive.joystickRemoteEvent()")
        print(message)
        if message['enable'] == True:
            cls.joystick['enabled'] = True
        elif message['enable'] == False:
            cls.joystick['enabled'] = False
    
    @classmethod
    async def remoteCall(cls, sid, message):
        """A function for an odrive being called remotely. The message
        will contain the odrive serial number, a unique identified
        that this script must return when the call completes, 
        the path to the function
        that should be called, and the value to pass to the function."""
        print("Inside MyOdrive.remoteCall()")
        print(message)
        try:
            request_id = message["request_id"]
            serial_number = message["serial_number"]
            # see if we have the odrive in our list that matches
            # the serial number.
            odrive = cls._odrives.get(serial_number)
            if odrive is None:
                # send a message back to the client that the odrive doesn't exist
                await cls._socketio.emit(request_id, 
                                    {"error":"odrive not found"})
                return
            print("Adding remote function call to the queue.")
            cls._remoteFunctionRequests.append(message)
        except Exception as ex:
            print("Exception in MyOdrive.remoteFunctionCall()")
            print(str(ex));
            await cls._socketio.emit(request_id, {"error":str(ex)})
       
    @classmethod
    def processRemoteRequests(cls):
        """This method is called from the main thread. It will process
        any remote requests that have been queued up by the socketio
        message handler."""
        print("Inside MyOdrive.processRemoteRequests()")
        while True:
            try:
                request_id = None
                while len(cls._remoteFunctionRequests) > 0:
                    message = cls._remoteFunctionRequests.pop(0)
                    print("Processing remote function call in thread")
                    request_id = message["request_id"]
                    serial_number = message["serial_number"]
                    function_name = message["function_name"]
                    arguments = message["args"]
                    tree = function_name.split(".")
                    function_name = tree.pop()
                    odrive = cls._odrives.get(serial_number)
                    property = odrive
                    for t in tree:
                        print(t)
                        p = getattr(property, t)
                        if issubclass(type(p), OdriveProperty):
                            property = p
                            
                    fun = getattr(property._reference, function_name)
                    if callable(fun):
                        print("Calling function " + function_name)
                        if function_name == "reboot" or function_name == "save_configuration":
                            odrive._reboot = True
                        result = None
                        if arguments is None:
                            result = fun()
                        else:
                            if type(arguments) is list:
                                result = fun(*arguments)
                            else:
                                result = fun(arguments)
                        msg = {"request_id":request_id, "result":result}
                        cls._socketSendQueue.append(msg)                                        
            except Exception as ex:
                print("Exception in MyOdrive.processRemoteRequests()")
                print(str(ex))
                cls.sendSocketMessageSync(request_id, {"error":str(ex)})
            time.sleep(0.5)
    
    @classmethod
    async def remotePropertySet(cls, sid, message):
        """A property is being set remotely. The message will contain
        the serial number for the odrive, the path to the property, and
        the value to set that property to.
        """
        print("Inside MyOdrive.remotePropertySet()")
        print(message)
        request_id = message["request_id"]
        serial_number = message["serial_number"]
        new_property = message["property_name"]
        value = message["value"]
        # see if we have the odrive in our list that matches
        # the serial number.
        try:
            odrive = cls._odrives.get(serial_number)
            if odrive is None:
                # send a message back to the client that the odrive doesn't exist
                await cls._socketio.emit(request_id, 
                                    {"error":"odrive not found"})
                return
            tree = new_property.split(".")
            new_property = tree.pop()
            odrive = cls._odrives.get(serial_number)
            property = odrive
            for t in tree:
                print(t)
                
                p= getattr(property, t)
                #if issubclass(type(p), OdriveProperty):
                property = p
                
                print("got property " + t)
                
            # set it here
            if issubclass(type(property), OdriveProperty):
                property = property._reference
                setattr(property, new_property, value)
            elif type(property) is dict:
                property[new_property] = value
            await cls._socketio.emit(request_id, {"result":"foobar"})
        except Exception as ex:
            print(ex)
            await cls._socketio.emit(request_id, {"error":str(ex)})
    
    @classmethod 
    async def connect(cls, sid, environ):
        print("Inside MyOdrive.socketio.connect(). Sending list of odrives...")
        await cls._socketio.emit("list_odrives", list(cls._odrives.keys()))
        print("Done")
        
    @classmethod
    async def addTelemetry(cls, sid, message):
        try:
            serial_number = message['serial_number']
            await cls._odrives[serial_number].addTelemetry(message['properties'])
        except Exception as ex:
            print("Failed to add telemetry")
            print(ex)
            
    
    @classmethod
    async def removeTelemetry(cls, sid, message):
        print("Foo1")
        try:
            serial_number = message['serial_number']
            await cls._odrives[serial_number].removeTelemetry(message['properties'])
        except Exception as ex:
            print("Failed to remove telemetry")
            print(ex)
    
    @classmethod
    async def handleSocketMessage(cls, sid, message):
        print("Inside the MyOdrive SocketIO message handler")
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
    async def startTelemetry(cls, sid, serial_number):
        print("MyOdrive.startTelemetry")
        print(serial_number)
        odrive = cls.getOdrive(serial_number)
        if odrive is not None:
            odrive._is_telemetry_running = True
            task = asyncio.create_task(odrive._telemetryTask())
            await task
        else:
            print("MyOdrive.startTelemetry failed to start telemetry")
    
    @classmethod
    def getOdrive(cls, serial_number) -> Odrive:
        if serial_number in cls._odrives.keys():
            return cls._odrives[serial_number]
        else:
            return None
    
    @classmethod
    async def onTelemetry(cls, serial_number, telem_obj):
        telem_obj['joystick_x'] = cls.joystick['x']
        telem_obj['joystick_y'] = cls.joystick['y']
        await cls._socketio.emit("telem", { str(serial_number): telem_obj })
    
    @classmethod
    async def stopTelemetry(cls, sid, serial_number):
        print("MyOdrive.stopTelemetry")
        print(serial_number)
        odrive = cls.getOdrive(serial_number)
        if odrive is not None:
            odrive._is_telemetry_running = False
        
    @classmethod
    async def syncOdrive(cls, sid, serial_number):
        print("MyOdrive.syncOdrive()")
        print(serial_number)
        try:
            if type(serial_number) is str:
                odrive = cls._odrives[serial_number]
                
                out = { serial_number: odrive._sync() }
                await cls._socketio.emit("sync", out)
        except Exception:
            print("Exception caught in MyOdrive.syncOdrive")
        
    
    @classmethod
    async def list_odrives(cls):
        #print("Inside static function show_odrives()")
        try:
            return list(cls._odrives.keys())
        except:
            return []
        
        
    @classmethod
    async def initialize(cls):
        task = asyncio.create_task(cls.asyncUpdate())
        await task
        
        
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
            #print("already have this odrive: " + str(serial) + ". Probably a reboot.")
            od = cls._odrives[serial];
            if od._reboot == True:
                print("resetting the reboot flag for an existing odrive.")
                # reset the reboot flag.
                odrive_handle = odrive.find_any(serial_number=serial)
                # replace the odrive handle in the odrive object
                od._odrive = odrive_handle
                od._reboot = False
                cls._broadcast_flag = True
            pass
        else:
            
            odrive_handle = odrive.find_any(serial_number=serial)
            hardware_version_major = getattr(odrive_handle, "hw_version_major")
            print("Newly detected Odrive hardware version major is: " + str(hardware_version_major))
            od = None
            if hardware_version_major == 3:
                od = Odrive3(serial_number=serial,
                             odrive_handle=odrive_handle)
                
                
            elif hardware_version_major == 4:
                od = Odrive4(serial_number=serial,
                            odrive_handle=odrive_handle)
                
            else:
                od = Odrive(serial_number=serial,
                        odrive_handle=odrive_handle)
            
            cls._odrives[serial] = od
            thread = Thread(target=od._thread_func)
            thread.start()
            cls._threads[serial] = thread
            
            # Notify connected socket.io clients of a new device.
            cls._broadcast_flag = True
    

    
