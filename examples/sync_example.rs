use em2rs::{Em2rsSyncClient, StepperConfig, Direction, PathConfig};
use tokio_modbus::prelude::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("EM2RS Sync Example");
    println!("==================\n");

    // Configure serial port
    let serial_port = "/dev/ttyUSB0";
    let baudrate = 9600;
    let slave_id = 1;
    
    println!("Opening serial port: {} at {} baud", serial_port, baudrate);
    let builder = tokio_serial::new(serial_port, baudrate);
    
    // Create synchronous Modbus RTU context (truly sync, no runtime!)
    println!("Creating synchronous Modbus RTU context for slave ID: {}", slave_id);
    let ctx = sync::rtu::connect_slave(&builder, Slave::from(slave_id))?;
    
    // Configure the stepper motor
    let config = StepperConfig::new(slave_id, 10000)
        .with_phase_current(2.0)
        .with_inductance(1000)
        .with_direction(Direction::Clockwise);
    
    // Create the synchronous client
    println!("Creating EM2RS sync client...");
    let mut motor = Em2rsSyncClient::new(ctx, config);
    
    // Initialize the motor (blocking call)
    println!("Initializing motor...");
    motor.init()?;
    println!("Motor initialized successfully!\n");
    
    // Get version (blocking)
    let version = motor.get_version()?;
    println!("Firmware version: {}", version);
    
    // Enable motor
    println!("\nEnabling motor...");
    motor.forced_enable_by_software(true)?;
    println!("Motor enabled");
    
    // Configure a path
    println!("\nConfiguring path 0...");
    let mut path = PathConfig::new(0)?;
    path.absolute_position = true;
    path.position = 5000;
    path.velocity = 300;
    path.acceleration = 150;
    path.deceleration = 150;
    
    motor.apply_path_config(&path)?;
    println!("Path 0 configured");
    
    // Start path (commented out for safety)
    // println!("Starting path 0...");
    // motor.start_path(0)?;
    // 
    // // Poll for completion (blocking)
    // loop {
    //     std::thread::sleep(std::time::Duration::from_millis(100));
    //     if motor.is_path_completed()? {
    //         println!("Path completed!");
    //         break;
    //     }
    // }
    
    // Get motion status
    println!("\nChecking motion status...");
    let status = motor.get_motion_status()?;
    println!("  Enabled: {}", status.is_enabled());
    println!("  Running: {}", status.is_running());
    println!("  Fault: {}", status.is_fault());
    
    // Jog motor example (commented out for safety)
    // println!("\nJogging motor clockwise...");
    // motor.jog_motor(Direction::Clockwise)?;
    // std::thread::sleep(std::time::Duration::from_secs(2));
    // motor.stop_motor()?;
    // println!("Motor stopped");
    
    println!("\nExample completed successfully!");
    
    Ok(())
}
