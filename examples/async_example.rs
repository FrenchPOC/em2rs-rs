use em2rs::{Em2rsClient, StepperConfig, Direction, PathConfig, HomingConfig, DigitalInputFunction, HomingMethod};
use tokio_modbus::prelude::*;
use tokio_serial::SerialStream;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("EM2RS Async Example");
    println!("===================\n");

    // Configure serial port
    let serial_port = "/dev/ttyUSB0";
    let baudrate = 9600;
    
    println!("Opening serial port: {} at {} baud", serial_port, baudrate);
    let builder = tokio_serial::new(serial_port, baudrate);
    let port = SerialStream::open(&builder)?;
    
    // Create Modbus RTU context
    let slave_id = 1;
    println!("Creating Modbus RTU context for slave ID: {}", slave_id);
    let ctx = rtu::attach_slave(port, Slave::from(slave_id));
    
    // Configure the stepper motor
    let config = StepperConfig::new(slave_id, 10000)
        .with_phase_current(2.0)
        .with_inductance(1000)
        .with_direction(Direction::Clockwise);
    
    // Create the client
    println!("Creating EM2RS client...");
    let mut motor = Em2rsClient::new(ctx, config);
    
    // Initialize the motor
    println!("Initializing motor...");
    motor.init().await?;
    println!("Motor initialized successfully!\n");
    
    // Get version
    let version = motor.get_version().await?;
    println!("Firmware version: {}", version);
    
    // Configure homing
    println!("\nConfiguring homing...");
    let homing_config = HomingConfig {
        input_no: 1,
        function: DigitalInputFunction::Org,
        normally_closed: false,
        direction: Direction::Clockwise,
        move_to_pos_after: true,
        method: HomingMethod::HomeSwitch,
        position: 0,
        position_stop: 1000,
        high_velocity: 200,
        low_velocity: 50,
        acceleration: 100,
        deceleration: 100,
    };
    
    motor.apply_homing_config(&homing_config).await?;
    println!("Homing configuration applied");
    
    // Start homing (commented out for safety)
    // println!("Starting homing sequence...");
    // motor.start_homing().await?;
    // 
    // // Wait for homing to complete
    // loop {
    //     tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    //     if motor.is_homing_completed().await? {
    //         println!("Homing completed!");
    //         break;
    //     }
    // }
    
    // Configure a path
    println!("\nConfiguring path 0...");
    let mut path = PathConfig::new(0)?;
    path.absolute_position = true;
    path.position = 5000;
    path.velocity = 300;
    path.acceleration = 150;
    path.deceleration = 150;
    path.pause_time = 500;
    
    motor.apply_path_config(&path).await?;
    println!("Path 0 configured");
    
    // Start path (commented out for safety)
    // println!("Starting path 0...");
    // motor.start_path(0).await?;
    // 
    // // Wait for path to complete
    // loop {
    //     tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    //     if motor.is_path_completed().await? {
    //         println!("Path completed!");
    //         break;
    //     }
    // }
    
    // Get motion status
    println!("\nChecking motion status...");
    let status = motor.get_motion_status().await?;
    println!("  Enabled: {}", status.is_enabled());
    println!("  Running: {}", status.is_running());
    println!("  Fault: {}", status.is_fault());
    
    // Save configuration to EEPROM (commented out for safety)
    // println!("\nSaving configuration to EEPROM...");
    // motor.save_param_eeprom().await?;
    // println!("Configuration saved");
    
    println!("\nExample completed successfully!");
    
    Ok(())
}
