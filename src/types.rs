use thiserror::Error;
use tokio_modbus::ExceptionCode;

/// Error types for EM2RS operations
#[derive(Error, Debug)]
pub enum Em2rsError {
    #[error("Modbus communication error: {0}")]
    Modbus(#[from] std::io::Error),
    
    #[error("Modbus protocol error: {0}")]
    ModbusProtocol(#[from] tokio_modbus::Error),
    
    #[error("Modbus exception: {0:?}")]
    ModbusException(#[from] ExceptionCode),
    
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),
    
    #[error("Invalid path ID: {0}. Must be 0-8")]
    InvalidPath(u8),
    
    #[error("Invalid digital input: {0}. Must be 1-7")]
    InvalidDigitalInput(u8),
    
    #[error("Operation failed: {0}")]
    OperationFailed(String),
}

pub type Result<T> = std::result::Result<T, Em2rsError>;

/// Motor rotation direction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum Direction {
    Clockwise = 0x00,
    CounterClockwise = 0x01,
}

impl From<Direction> for u16 {
    fn from(dir: Direction) -> Self {
        dir as u16
    }
}

/// Digital input configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum DigitalInputFunction {
    Invalid = 0x00,
    AlarmClearing = 0x07,
    Enable = 0x08,
    TriggerCmd = 0x20,
    TriggerHoming = 0x21,
    Emergency = 0x22,
    JogPositive = 0x23,
    JogNegative = 0x24,
    Pot = 0x25,
    Not = 0x26,
    Org = 0x27,
    Add0 = 0x28,
    Add1 = 0x29,
    Add2 = 0x2A,
    Add3 = 0x2B,
    JogVelocity = 0x2C,
}

impl From<DigitalInputFunction> for u16 {
    fn from(func: DigitalInputFunction) -> Self {
        func as u16
    }
}

/// Digital output configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum DigitalOutputFunction {
    Invalid = 0x00,
    CmdCompleted = 0x20,
    PathCompleted = 0x21,
    HomingCompleted = 0x22,
    InPosCompleted = 0x23,
    BrakeOutput = 0x24,
    AlarmOutput = 0x25,
}

impl From<DigitalOutputFunction> for u16 {
    fn from(func: DigitalOutputFunction) -> Self {
        func as u16
    }
}

/// Control word commands
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ControlWord {
    ResetCurrentAlarm = 0x1111,
    ResetHistoryAlarm = 0x1122,
    SaveParamEeprom = 0x2211,
    ParamReset = 0x2222,
    FactoryReset = 0x2233,
    SaveMappingEeprom = 0x2244,
    JogClockwise = 0x4001,
    JogCounterClockwise = 0x4002,
}

impl From<ControlWord> for u16 {
    fn from(cw: ControlWord) -> Self {
        cw as u16
    }
}

/// Save parameter status word
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum SaveParameterStatus {
    SaveSuccessfully = 0x5555,
    FailedToSave = 0xAAAA,
}

/// Current alarm flags
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentAlarm(pub u16);

impl CurrentAlarm {
    pub const OVER_CURRENT: u16 = 0x01;
    pub const OVER_VOLTAGE: u16 = 0x02;
    pub const CURRENT_SAMPLING_FAULT: u16 = 0x40;
    pub const FAILED_LOCK_SHAFT: u16 = 0x80;
    pub const EEPROM_FAULT: u16 = 0x200;
    pub const AUTOTUNING_FAULT: u16 = 0x100;

    pub fn has_over_current(&self) -> bool {
        self.0 & Self::OVER_CURRENT != 0
    }

    pub fn has_over_voltage(&self) -> bool {
        self.0 & Self::OVER_VOLTAGE != 0
    }

    pub fn has_current_sampling_fault(&self) -> bool {
        self.0 & Self::CURRENT_SAMPLING_FAULT != 0
    }

    pub fn has_failed_lock_shaft(&self) -> bool {
        self.0 & Self::FAILED_LOCK_SHAFT != 0
    }

    pub fn has_eeprom_fault(&self) -> bool {
        self.0 & Self::EEPROM_FAULT != 0
    }

    pub fn has_autotuning_fault(&self) -> bool {
        self.0 & Self::AUTOTUNING_FAULT != 0
    }
}

/// Homing method
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum HomingMethod {
    LimitSwitch = 0x00,
    HomeSwitch = 0x04,
}

impl From<HomingMethod> for u16 {
    fn from(method: HomingMethod) -> Self {
        method as u16
    }
}

/// PR control register commands
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum PrControlCommand {
    RunThePath = 0x10,
    Homing = 0x20,
    ManualZero = 0x21,
    QuickStop = 0x40,
}

impl From<PrControlCommand> for u16 {
    fn from(cmd: PrControlCommand) -> Self {
        cmd as u16
    }
}

/// Path motion type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum PathMotionType {
    NoAction = 0x00,
    PositionPositioning = 0x01,
    VelocityMovement = 0x02,
    Homing = 0x03,
}

impl From<PathMotionType> for u16 {
    fn from(pmt: PathMotionType) -> Self {
        pmt as u16
    }
}

/// Motion status flags
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotionStatus(pub u16);

impl MotionStatus {
    pub fn is_fault(&self) -> bool {
        self.0 & crate::registers::flags::MS_FAULT != 0
    }

    pub fn is_enabled(&self) -> bool {
        self.0 & crate::registers::flags::MS_ENABLE != 0
    }

    pub fn is_running(&self) -> bool {
        self.0 & crate::registers::flags::MS_RUNNING != 0
    }

    pub fn is_cmd_complete(&self) -> bool {
        self.0 & crate::registers::flags::MS_CMD_COMPLETE != 0
    }

    pub fn is_path_complete(&self) -> bool {
        self.0 & crate::registers::flags::MS_PATH_COMPLETE != 0
    }

    pub fn is_homing_complete(&self) -> bool {
        self.0 & crate::registers::flags::MS_HOMING_COMPLETE != 0
    }
}

/// Homing configuration
#[derive(Debug, Clone)]
pub struct HomingConfig {
    pub input_no: u8,
    pub function: DigitalInputFunction,
    pub normally_closed: bool,
    pub direction: Direction,
    pub move_to_pos_after: bool,
    pub method: HomingMethod,
    pub position: u32,
    pub position_stop: u32,
    pub high_velocity: u16,
    pub low_velocity: u16,
    pub acceleration: u16,
    pub deceleration: u16,
}

impl Default for HomingConfig {
    fn default() -> Self {
        Self {
            input_no: 1,
            function: DigitalInputFunction::Org,
            normally_closed: false,
            direction: Direction::Clockwise,
            move_to_pos_after: true,
            method: HomingMethod::HomeSwitch,
            position: 0,
            position_stop: 0,
            high_velocity: 100,
            low_velocity: 50,
            acceleration: 100,
            deceleration: 100,
        }
    }
}

/// Path configuration
#[derive(Debug, Clone)]
pub struct PathConfig {
    pub path_id: u8,
    pub absolute_position: bool,
    pub position: u32,
    pub velocity: u16,
    pub acceleration: u16,
    pub deceleration: u16,
    pub pause_time: u16,
}

impl PathConfig {
    pub fn new(path_id: u8) -> Result<Self> {
        if path_id > 8 {
            return Err(Em2rsError::InvalidPath(path_id));
        }
        Ok(Self {
            path_id,
            absolute_position: true,
            position: 0,
            velocity: 100,
            acceleration: 100,
            deceleration: 100,
            pause_time: 0,
        })
    }
}

/// Stepper motor configuration
#[derive(Debug, Clone)]
pub struct StepperConfig {
    pub slave_id: u8,
    pub pulse_per_rev: u16,
    pub direction: Direction,
    pub phase_current: f32,
    pub inductance: u16,
}

impl StepperConfig {
    pub fn new(slave_id: u8, pulse_per_rev: u16) -> Self {
        Self {
            slave_id,
            pulse_per_rev,
            direction: Direction::Clockwise,
            phase_current: 1.0,
            inductance: 1000,
        }
    }

    pub fn with_phase_current(mut self, current: f32) -> Self {
        self.phase_current = current;
        self
    }

    pub fn with_inductance(mut self, inductance: u16) -> Self {
        self.inductance = inductance;
        self
    }

    pub fn with_direction(mut self, direction: Direction) -> Self {
        self.direction = direction;
        self
    }
}
