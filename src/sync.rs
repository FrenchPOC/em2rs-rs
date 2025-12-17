#[cfg(feature = "modbus-delay")]
use std::thread;
#[cfg(feature = "modbus-delay")]
use std::time::Duration;
use tokio_modbus::prelude::*;
use crate::registers;
use crate::registers::flags;
use crate::types::*;

/// Default delay after modbus requests (1ms)
#[cfg(feature = "modbus-delay")]
const MODBUS_DELAY: Duration = Duration::from_millis(1);

/// Synchronous EM2RS stepper motor controller client
/// 
/// This client uses tokio-modbus sync API for blocking Modbus RTU communication.
/// Multiple instances can be created for different motor IDs on the same bus.
pub struct Em2rsSyncClient {
    ctx: client::sync::Context,
    slave_id: u8,
    config: StepperConfig,
}

impl Em2rsSyncClient {
    /// Create a new synchronous EM2RS client
    /// 
    /// # Arguments
    /// * `ctx` - Tokio-modbus sync context (already initialized for RTU communication)
    /// * `config` - Stepper motor configuration including slave ID
    pub fn new(ctx: client::sync::Context, config: StepperConfig) -> Self {
        Self {
            ctx,
            slave_id: config.slave_id,
            config,
        }
    }

    /// Consume the client and return the underlying Modbus context
    /// 
    /// This is useful when you want to reuse the same physical connection
    /// for multiple motors on the same RS485 bus with different slave IDs.
    pub fn into_context(self) -> client::sync::Context {
        self.ctx
    }

    /// Initialize the stepper motor with configured parameters
    pub fn init(&mut self) -> Result<()> {
        self.ctx.set_slave(Slave::from(self.slave_id));
        
        // Set pulse per revolution
        self.write_register(registers::PULSE_PER_REV, self.config.pulse_per_rev)?;
        
        // Set motor direction
        self.write_register(registers::MOTOR_DIRECTION, self.config.direction.into())?;
        
        // Set peak current
        self.set_peak_current(self.config.phase_current)?;
        
        // Set motor inductance
        self.set_motor_inductance(self.config.inductance)?;
        
        Ok(())
    }

    /// Write a single holding register
    fn write_register(&mut self, addr: u16, value: u16) -> Result<()> {
        let _ = self.ctx.write_single_register(addr, value)?;
        #[cfg(feature = "modbus-delay")]
        thread::sleep(MODBUS_DELAY);
        Ok(())
    }

    /// Write multiple holding registers
    #[allow(dead_code)]
    fn write_registers(&mut self, addr: u16, values: &[u16]) -> Result<()> {
        let _ = self.ctx.write_multiple_registers(addr, values)?;
        #[cfg(feature = "modbus-delay")]
        thread::sleep(MODBUS_DELAY);
        Ok(())
    }

    /// Read holding registers
    fn read_registers(&mut self, addr: u16, count: u16) -> Result<Vec<u16>> {
        let data = self.ctx.read_holding_registers(addr, count)??;
        #[cfg(feature = "modbus-delay")]
        thread::sleep(MODBUS_DELAY);
        Ok(data)
    }

    /// Set peak current based on phase current
    /// Peak current = phase_current * 1.4 * 10
    pub fn set_peak_current(&mut self, phase_current: f32) -> Result<()> {
        let peak_current = (phase_current * 1.4 * 10.0) as u16;
        self.write_register(registers::PEAK_CURRENT, peak_current)
    }

    /// Set motor inductance (max 10000)
    pub fn set_motor_inductance(&mut self, inductance: u16) -> Result<()> {
        let ind = inductance.min(10000);
        self.write_register(registers::MOTOR_INDUCTANCE, ind)
    }

    /// Enable or disable forced software enable
    pub fn forced_enable_by_software(&mut self, enable: bool) -> Result<()> {
        let value = if enable { 0x0001 } else { 0x0000 };
        self.write_register(registers::FORCED_ENA, value)
    }

    /// Send a control word command
    fn set_control_word(&mut self, command: ControlWord) -> Result<()> {
        self.write_register(registers::CONTROL_WORD, command.into())
    }

    /// Save parameters to EEPROM
    pub fn save_param_eeprom(&mut self) -> Result<()> {
        self.set_control_word(ControlWord::SaveParamEeprom)
    }

    /// Reset parameters (excluding motor parameters)
    pub fn param_reset(&mut self) -> Result<()> {
        self.set_control_word(ControlWord::ParamReset)
    }

    /// Factory reset
    pub fn factory_reset(&mut self) -> Result<()> {
        self.set_control_word(ControlWord::FactoryReset)
    }

    /// Save I/O mapping to EEPROM
    pub fn save_mapping_eeprom(&mut self) -> Result<()> {
        self.set_control_word(ControlWord::SaveMappingEeprom)
    }

    /// Jog the motor in specified direction
    pub fn jog_motor(&mut self, direction: Direction) -> Result<()> {
        let command = match direction {
            Direction::Clockwise => ControlWord::JogClockwise,
            Direction::CounterClockwise => ControlWord::JogCounterClockwise,
        };
        self.set_control_word(command)
    }

    /// Configure a digital input
    pub fn configure_input(
        &mut self,
        input_no: u8,
        function: DigitalInputFunction,
        normally_closed: bool,
    ) -> Result<()> {
        if !(1..=7).contains(&input_no) {
            return Err(Em2rsError::InvalidDigitalInput(input_no));
        }

        let config = u16::from(function) + if normally_closed { flags::SI_NC_INCR } else { 0 };
        let register = registers::SI1 + ((input_no - 1) as u16 * 2);
        self.write_register(register, config)
    }

    /// Get digital input status
    pub fn get_input_status(&mut self) -> Result<u16> {
        let data = self.read_registers(registers::DIGITAL_INPUT_STATUS, 1)?;
        Ok(data[0])
    }

    /// Get motion status
    pub fn get_motion_status(&mut self) -> Result<MotionStatus> {
        let data = self.read_registers(registers::MOTION_STATUS, 1)?;
        Ok(MotionStatus(data[0]))
    }

    /// Check if path is completed
    pub fn is_path_completed(&mut self) -> Result<bool> {
        let status = self.get_motion_status()?;
        Ok(status.is_path_complete())
    }

    /// Check if homing is completed
    pub fn is_homing_completed(&mut self) -> Result<bool> {
        let status = self.get_motion_status()?;
        Ok(status.is_homing_complete())
    }

    /// Set CTRG effective edge (double edge or single)
    pub fn set_ctrg_effective_edge(&mut self, double_edge: bool) -> Result<()> {
        let mut reg = self.read_registers(registers::PR_GLOBAL_CTRL_FCT, 1)?[0];
        if double_edge {
            reg |= 1 << 0;
        } else {
            reg &= !(1 << 0);
        }
        self.write_register(registers::PR_GLOBAL_CTRL_FCT, reg)
    }

    /// Enable or disable soft limit control
    pub fn soft_limit_control(&mut self, enable: bool) -> Result<()> {
        let mut reg = self.read_registers(registers::PR_GLOBAL_CTRL_FCT, 1)?[0];
        if enable {
            reg |= 1 << 1;
        } else {
            reg &= !(1 << 1);
        }
        self.write_register(registers::PR_GLOBAL_CTRL_FCT, reg)
    }

    /// Set soft limit maximum position
    pub fn set_soft_limit_max(&mut self, max: u32) -> Result<()> {
        let lsb = (max & 0xFFFF) as u16;
        let msb = ((max >> 16) & 0xFFFF) as u16;
        self.write_register(registers::SOFT_LIMIT_P_H, msb)?;
        self.write_register(registers::SOFT_LIMIT_P_L, lsb)
    }

    /// Set soft limit minimum position
    pub fn set_soft_limit_min(&mut self, min: u32) -> Result<()> {
        let lsb = (min & 0xFFFF) as u16;
        let msb = ((min >> 16) & 0xFFFF) as u16;
        self.write_register(registers::SOFT_LIMIT_N_H, msb)?;
        self.write_register(registers::SOFT_LIMIT_N_L, lsb)
    }

    /// Enable or disable homing on power up
    pub fn homing_power_up_control(&mut self, enable: bool) -> Result<()> {
        let mut reg = self.read_registers(registers::PR_GLOBAL_CTRL_FCT, 1)?[0];
        if enable {
            reg |= 1 << 2;
        } else {
            reg &= !(1 << 2);
        }
        self.write_register(registers::PR_GLOBAL_CTRL_FCT, reg)
    }

    /// Configure CTRG trigger type (0: Bit0, 1: Level Trigger)
    pub fn set_ctrg_trigger_type(&mut self, level_trigger: bool) -> Result<()> {
        let mut reg = self.read_registers(registers::PR_GLOBAL_CTRL_FCT, 1)?[0];
        if level_trigger {
            reg |= 1 << 4;
        } else {
            reg &= !(1 << 4);
        }
        self.write_register(registers::PR_GLOBAL_CTRL_FCT, reg)
    }

    /// Configure homing parameters
    pub fn configure_homing(
        &mut self,
        direction: Direction,
        move_to_pos: bool,
        method: HomingMethod,
    ) -> Result<()> {
        let config = u16::from(direction) 
            + if move_to_pos { 0x0002 } else { 0x0000 } 
            + u16::from(method);
        self.write_register(registers::HOME_MODE, config)?;
        self.write_register(0x601A, 0x0002)  // Additional configuration
    }

    /// Set homing switch position
    pub fn set_homing_position(&mut self, position: u32) -> Result<()> {
        let lsb = (position & 0xFFFF) as u16;
        let msb = ((position >> 16) & 0xFFFF) as u16;
        self.write_register(registers::HOME_SWITCH_POS_HIGH, msb)?;
        self.write_register(registers::HOME_SWITCH_POS_LOW, lsb)
    }

    /// Set homing stop position
    pub fn set_homing_stop_position(&mut self, position: u32) -> Result<()> {
        let lsb = (position & 0xFFFF) as u16;
        let msb = ((position >> 16) & 0xFFFF) as u16;
        self.write_register(registers::HOMING_STOP_POS_HIGH, msb)?;
        self.write_register(registers::HOMING_STOP_POS_LOW, lsb)
    }

    /// Set homing high velocity (RPM)
    pub fn set_homing_high_velocity(&mut self, rpm: u16) -> Result<()> {
        self.write_register(registers::HOMING_HIGH_VELOCITY, rpm)
    }

    /// Set homing low velocity (RPM)
    pub fn set_homing_low_velocity(&mut self, rpm: u16) -> Result<()> {
        self.write_register(registers::HOMING_LOW_VELOCITY, rpm)
    }

    /// Set homing acceleration
    pub fn set_homing_acceleration(&mut self, acc: u16) -> Result<()> {
        self.write_register(registers::HOMING_ACC, acc)
    }

    /// Set homing deceleration
    pub fn set_homing_deceleration(&mut self, dec: u16) -> Result<()> {
        self.write_register(registers::HOMING_DEC, dec)
    }

    /// Apply complete homing configuration
    pub fn apply_homing_config(&mut self, config: &HomingConfig) -> Result<()> {
        self.configure_input(config.input_no, config.function, config.normally_closed)?;
        self.configure_homing(config.direction, config.move_to_pos_after, config.method)?;
        self.set_homing_position(config.position)?;
        self.set_homing_stop_position(config.position_stop)?;
        self.set_homing_high_velocity(config.high_velocity)?;
        self.set_homing_low_velocity(config.low_velocity)?;
        self.set_homing_acceleration(config.acceleration)?;
        self.set_homing_deceleration(config.deceleration)?;
        Ok(())
    }

    /// Send PR control command
    fn set_pr_control(&mut self, command: PrControlCommand) -> Result<()> {
        self.write_register(registers::PR_CTRL, command.into())
    }

    /// Start homing sequence
    pub fn start_homing(&mut self) -> Result<()> {
        self.set_pr_control(PrControlCommand::Homing)
    }

    /// Start a path (0-8)
    pub fn start_path(&mut self, path_id: u8) -> Result<()> {
        if path_id > 8 {
            return Err(Em2rsError::InvalidPath(path_id));
        }
        let command_value = u16::from(PrControlCommand::RunThePath) + path_id as u16;
        self.write_register(registers::PR_CTRL, command_value)
    }

    /// Quick stop the motor
    pub fn stop_motor(&mut self) -> Result<()> {
        self.set_pr_control(PrControlCommand::QuickStop)
    }

    /// Set current position as zero
    pub fn manual_zero(&mut self) -> Result<()> {
        self.set_pr_control(PrControlCommand::ManualZero)
    }

    /// Configure path motion parameters
    /// 
    /// For simpler usage, consider using `apply_path_config` with a `PathConfig` struct
    #[allow(clippy::too_many_arguments)]
    pub fn configure_path_motion(
        &mut self,
        path_id: u8,
        motion_type: PathMotionType,
        interrupt: bool,
        overlap: bool,
        absolute: bool,
        jump: bool,
        jump_to: u8,
    ) -> Result<()> {
        let base = registers::get_path_base(path_id).ok_or(Em2rsError::InvalidPath(path_id))?;
        
        let mut config = u16::from(motion_type)
            + if interrupt { 0x0010 } else { 0x0000 }
            + if overlap { 0x0020 } else { 0x0000 }
            + if absolute { 0x0000 } else { 0x0040 };
        
        if jump {
            config += 0x4000 + (((jump_to & 0x0F) as u16) << 8);
        }
        
        self.write_register(base, config)
    }

    /// Set path position (32-bit)
    pub fn set_path_position(&mut self, path_id: u8, position: u32) -> Result<()> {
        let base = registers::get_path_base(path_id).ok_or(Em2rsError::InvalidPath(path_id))?;
        let lsb = (position & 0xFFFF) as u16;
        let msb = ((position >> 16) & 0xFFFF) as u16;
        
        self.write_register(base + registers::PATH_POSITION_H_OFFSET, msb)?;
        self.write_register(base + registers::PATH_POSITION_L_OFFSET, lsb)
    }

    /// Set path velocity (RPM)
    pub fn set_path_velocity(&mut self, path_id: u8, rpm: u16) -> Result<()> {
        let base = registers::get_path_base(path_id).ok_or(Em2rsError::InvalidPath(path_id))?;
        self.write_register(base + registers::PATH_VELOCITY_OFFSET, rpm)
    }

    /// Set path acceleration (ms/1000rpm)
    pub fn set_path_acceleration(&mut self, path_id: u8, acc: u16) -> Result<()> {
        let base = registers::get_path_base(path_id).ok_or(Em2rsError::InvalidPath(path_id))?;
        self.write_register(base + registers::PATH_ACC_OFFSET, acc)
    }

    /// Set path deceleration (ms/1000rpm)
    pub fn set_path_deceleration(&mut self, path_id: u8, dec: u16) -> Result<()> {
        let base = registers::get_path_base(path_id).ok_or(Em2rsError::InvalidPath(path_id))?;
        self.write_register(base + registers::PATH_DEC_OFFSET, dec)
    }

    /// Set path pause time (ms)
    pub fn set_path_pause_time(&mut self, path_id: u8, ms: u16) -> Result<()> {
        let base = registers::get_path_base(path_id).ok_or(Em2rsError::InvalidPath(path_id))?;
        self.write_register(base + registers::PATH_PAUSE_TIME_OFFSET, ms)
    }

    /// Apply complete path configuration
    pub fn apply_path_config(&mut self, config: &PathConfig) -> Result<()> {
        self.configure_path_motion(
            config.path_id,
            PathMotionType::PositionPositioning,
            false,
            false,
            config.absolute_position,
            false,
            0,
        )?;
        
        self.set_path_position(config.path_id, config.position)?;
        self.set_path_velocity(config.path_id, config.velocity)?;
        self.set_path_acceleration(config.path_id, config.acceleration)?;
        self.set_path_deceleration(config.path_id, config.deceleration)?;
        
        if config.pause_time > 0 {
            self.set_path_pause_time(config.path_id, config.pause_time)?;
        }
        
        Ok(())
    }

    /// Get firmware version
    pub fn get_version(&mut self) -> Result<u16> {
        let data = self.read_registers(registers::VERSION_INFORMATION, 1)?;
        Ok(data[0])
    }

    /// Get current alarm status
    pub fn get_current_alarm(&mut self) -> Result<CurrentAlarm> {
        let data = self.read_registers(registers::CURRENT_ALARM, 1)?;
        Ok(CurrentAlarm(data[0]))
    }
}
