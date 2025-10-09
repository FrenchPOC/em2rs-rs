/// Register addresses for EM2RS stepper motor controller
// Basic Parameters Registers
pub const PULSE_PER_REV: u16 = 0x0001;
pub const CONTROL_MODE_SOURCE: u16 = 0x0005;
pub const MOTOR_DIRECTION: u16 = 0x0007;
pub const MOTOR_INDUCTANCE: u16 = 0x0009;
pub const FORCED_ENA: u16 = 0x000F;
pub const CMD_FILTER_TIME: u16 = 0x00A1;

// Digital Input Configuration (0x0145 - 0x0151)
pub const SI1: u16 = 0x0145;
pub const SI2: u16 = 0x0147;
pub const SI3: u16 = 0x0149;
pub const SI4: u16 = 0x014B;
pub const SI5: u16 = 0x014D;
pub const SI6: u16 = 0x014F;
pub const SI7: u16 = 0x0151;

// Digital Output Configuration (0x0157 - 0x015B)
pub const SO1: u16 = 0x0157;
pub const SO2: u16 = 0x0159;
pub const SO3: u16 = 0x015B;

// Brake and Alarm Configuration
pub const DELAY_BRAKE_RELEASED: u16 = 0x0167;
pub const DELAY_BRAKE_LOCKED: u16 = 0x0169;
pub const THRESHOLD_BRAKE: u16 = 0x016B;
pub const ALARM_DETECTION: u16 = 0x016D;

// Status Registers
pub const BUS_VOLTAGE: u16 = 0x0177;
pub const DIGITAL_INPUT_STATUS: u16 = 0x0179;
pub const DIGITAL_OUTPUT_STATUS: u16 = 0x017B;
pub const DIP_SW_STATUS: u16 = 0x0187;

// Motor Parameters
pub const PEAK_CURRENT: u16 = 0x0191;
pub const PERCENT_SHAFT_LOCKED: u16 = 0x0197;
pub const SHAFT_LOCKED_DURATION: u16 = 0x0199;
pub const SHAFT_LOCKED_RISING_TIME: u16 = 0x019F;
pub const MAX_STOP_TIME: u16 = 0x01A5;
pub const AUTO_TUNING_POWER_ON: u16 = 0x01AB;

// RS485 Configuration
pub const RS485_BAUDRATE: u16 = 0x01BD;
pub const RS485_ID: u16 = 0x01BF;
pub const RS485_DATA_TYPE: u16 = 0x01C1;
pub const RS485_CONTROL_WORD: u16 = 0x01C3;
pub const COM_BIT_DELAY: u16 = 0x01C4;

// Standby Configuration
pub const SWITCHING_TIME_STANDBY: u16 = 0x01D1;
pub const STANDBY_CURRENT_PERCENT: u16 = 0x01D3;

// Jog Configuration
pub const JOG_VELOCITY: u16 = 0x01E1;
pub const INTERVAL: u16 = 0x01E3;
pub const RUNNING_TIME: u16 = 0x01E5;
pub const ACC_DEC_TIME: u16 = 0x01E7;

// Version and Firmware
pub const VERSION_INFORMATION: u16 = 0x01FF;
pub const FIRMWARE_INFORMATION: u16 = 0x0201;

// Motor Model and Advanced Parameters
pub const MOTOR_MODEL: u16 = 0x0231;
pub const BACK_EMF_COEF: u16 = 0x0235;
pub const CURRENT_LOOP_PROPORTIONAL_KP: u16 = 0x0237;
pub const CURRENT_LOOP_KI: u16 = 0x0239;
pub const CURRENT_LOOP_KP: u16 = 0x023B;
pub const CURRENT_LOOP_KC: u16 = 0x023D;
pub const OVER_VOLTAGE_THRESHOLD: u16 = 0x0243;

// Motion Status and Control
pub const MOTION_STATUS: u16 = 0x1003;
pub const CONTROL_WORD: u16 = 0x1801;
pub const SAVE_PARAMETER_STATUS_WORD: u16 = 0x1901;
pub const CURRENT_ALARM: u16 = 0x2203;

// PR (Position/Routine) Control
pub const PR_GLOBAL_CTRL_FCT: u16 = 0x6000;
pub const PR_CTRL: u16 = 0x6002;
pub const SOFT_LIMIT_P_H: u16 = 0x6006;
pub const SOFT_LIMIT_P_L: u16 = 0x6007;
pub const SOFT_LIMIT_N_H: u16 = 0x6008;
pub const SOFT_LIMIT_N_L: u16 = 0x6009;

// Homing Configuration
pub const HOME_MODE: u16 = 0x600A;
pub const HOME_SWITCH_POS_HIGH: u16 = 0x600B;
pub const HOME_SWITCH_POS_LOW: u16 = 0x600C;
pub const HOMING_STOP_POS_HIGH: u16 = 0x600D;
pub const HOMING_STOP_POS_LOW: u16 = 0x600E;
pub const HOMING_HIGH_VELOCITY: u16 = 0x600F;
pub const HOMING_LOW_VELOCITY: u16 = 0x6010;
pub const HOMING_ACC: u16 = 0x6011;
pub const HOMING_DEC: u16 = 0x6012;

// Path Configuration Base Addresses
pub const PATH0_BASE: u16 = 0x6200;
pub const PATH1_BASE: u16 = 0x6208;
pub const PATH2_BASE: u16 = 0x6210;
pub const PATH3_BASE: u16 = 0x6218;
pub const PATH4_BASE: u16 = 0x6220;
pub const PATH5_BASE: u16 = 0x6228;
pub const PATH6_BASE: u16 = 0x6230;
pub const PATH7_BASE: u16 = 0x6238;
pub const PATH8_BASE: u16 = 0x6240;

// Path register offsets from base
pub const PATH_CTRL_OFFSET: u16 = 0;
pub const PATH_POSITION_H_OFFSET: u16 = 1;
pub const PATH_POSITION_L_OFFSET: u16 = 2;
pub const PATH_VELOCITY_OFFSET: u16 = 3;
pub const PATH_ACC_OFFSET: u16 = 4;
pub const PATH_DEC_OFFSET: u16 = 5;
pub const PATH_PAUSE_TIME_OFFSET: u16 = 6;
pub const PATH_SPECIAL_PARAM_OFFSET: u16 = 7;

/// Bit flags and increments
pub mod flags {
    // Digital input normally closed increment
    pub const SI_NC_INCR: u16 = 0x0080;
    
    // Digital output normally closed increment
    pub const SO_NC_INCR: u16 = 0x0080;

    // Motion Status flags
    pub const MS_FAULT: u16 = 0x0001;
    pub const MS_ENABLE: u16 = 0x0002;
    pub const MS_RUNNING: u16 = 0x0004;
    pub const MS_CMD_COMPLETE: u16 = 0x0010;
    pub const MS_PATH_COMPLETE: u16 = 0x0020;
    pub const MS_HOMING_COMPLETE: u16 = 0x0040;
}

/// Helper function to get path base register
pub const fn get_path_base(path_id: u8) -> Option<u16> {
    match path_id {
        0 => Some(PATH0_BASE),
        1 => Some(PATH1_BASE),
        2 => Some(PATH2_BASE),
        3 => Some(PATH3_BASE),
        4 => Some(PATH4_BASE),
        5 => Some(PATH5_BASE),
        6 => Some(PATH6_BASE),
        7 => Some(PATH7_BASE),
        8 => Some(PATH8_BASE),
        _ => None,
    }
}
