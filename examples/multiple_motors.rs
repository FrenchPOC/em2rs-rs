/// Example showing multiple motor instances on the same RS485 bus
/// 
/// This example demonstrates how to control multiple motors connected to the same
/// physical RS485 bus. All motors share the same serial port but have different
/// slave IDs. The client switches between slave IDs dynamically when communicating
/// with each motor.
/// 
/// Each motor must have a unique slave ID configured in its hardware settings.
use em2rs::{Em2rsClient, StepperConfig, Direction, PathConfig};
use tokio_modbus::prelude::*;
use tokio_serial::SerialStream;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("EM2RS Multiple Motors Example");
    println!("==============================");
    println!("This example shows 3 motors on ONE physical RS485 bus\n");

    // Open the serial port ONCE - this is the single physical RS485 bus
    let serial_port = "/dev/ttyUSB0";
    let baudrate = 9600;
    
    println!("Opening RS485 bus: {} at {} baud", serial_port, baudrate);
    let builder = tokio_serial::new(serial_port, baudrate);
    let port = SerialStream::open(&builder)?;
    
    // Start with motor 1 context
    let ctx = rtu::attach_slave(port, Slave::from(1));
    
    // Create Motor 1 (Slave ID 1)
    println!("\n=== Motor 1 (Slave ID 1) ===");
    let config1 = StepperConfig::new(1, 10000)
        .with_phase_current(2.0)
        .with_inductance(1000)
        .with_direction(Direction::Clockwise);
    
    let mut motor1 = Em2rsClient::new(ctx, config1);
    println!("Initializing motor 1...");
    motor1.init().await?;
    let version1 = motor1.get_version().await?;
    println!("Motor 1 initialized! Version: {}", version1);
    
    // Configure path for Motor 1
    let mut path1 = PathConfig::new(0)?;
    path1.absolute_position = true;
    path1.position = 5000;
    path1.velocity = 300;
    path1.acceleration = 150;
    path1.deceleration = 150;
    
    println!("Configuring path for motor 1...");
    motor1.apply_path_config(&path1).await?;
    
    // Get status from Motor 1
    let status1 = motor1.get_motion_status().await?;
    println!("Motor 1 Status - Enabled: {}, Running: {}, Fault: {}", 
             status1.is_enabled(), status1.is_running(), status1.is_fault());
    
    // Now switch to Motor 2 (Slave ID 2) on the SAME bus
    println!("\n=== Motor 2 (Slave ID 2) ===");
    println!("Switching to motor 2 on the same bus...");
    
    let config2 = StepperConfig::new(2, 10000)
        .with_phase_current(1.5)
        .with_inductance(800)
        .with_direction(Direction::CounterClockwise);
    
    // Extract the context from motor1 and reuse it for motor2
    let ctx = motor1.into_context();
    let mut motor2 = Em2rsClient::new(ctx, config2);
    
    println!("Initializing motor 2...");
    motor2.init().await?;
    let version2 = motor2.get_version().await?;
    println!("Motor 2 initialized! Version: {}", version2);
    
    // Configure path for Motor 2
    let mut path2 = PathConfig::new(0)?;
    path2.absolute_position = true;
    path2.position = 8000;
    path2.velocity = 400;
    path2.acceleration = 200;
    path2.deceleration = 200;
    
    println!("Configuring path for motor 2...");
    motor2.apply_path_config(&path2).await?;
    
    // Get status from Motor 2
    let status2 = motor2.get_motion_status().await?;
    println!("Motor 2 Status - Enabled: {}, Running: {}, Fault: {}", 
             status2.is_enabled(), status2.is_running(), status2.is_fault());
    
    // Now switch to Motor 3 (Slave ID 3) on the SAME bus
    println!("\n=== Motor 3 (Slave ID 3) ===");
    println!("Switching to motor 3 on the same bus...");
    
    let config3 = StepperConfig::new(3, 10000)
        .with_phase_current(2.5)
        .with_inductance(1200)
        .with_direction(Direction::Clockwise);
    
    // Extract the context from motor2 and reuse it for motor3
    let ctx = motor2.into_context();
    let mut motor3 = Em2rsClient::new(ctx, config3);
    
    println!("Initializing motor 3...");
    motor3.init().await?;
    let version3 = motor3.get_version().await?;
    println!("Motor 3 initialized! Version: {}", version3);
    
    // Configure path for Motor 3
    let mut path3 = PathConfig::new(0)?;
    path3.absolute_position = true;
    path3.position = 3000;
    path3.velocity = 250;
    path3.acceleration = 100;
    path3.deceleration = 100;
    
    println!("Configuring path for motor 3...");
    motor3.apply_path_config(&path3).await?;
    
    // Get status from Motor 3
    let status3 = motor3.get_motion_status().await?;
    println!("Motor 3 Status - Enabled: {}, Running: {}, Fault: {}", 
             status3.is_enabled(), status3.is_running(), status3.is_fault());
    
    // Example: You can continue switching between motors as needed
    // To go back to motor 1, you would do:
    // let ctx = motor3.into_context();
    // let mut motor1_again = Em2rsClient::new(ctx, config1);
    // motor1_again.start_path(0).await?;
    
    println!("\n=== Example completed successfully! ===");
    println!("This example demonstrated:");
    println!("  ✓ ONE physical RS485 bus ({serial_port})");
    println!("  ✓ THREE motors with slave IDs 1, 2, and 3");
    println!("  ✓ Sequential communication by reusing the same connection");
    println!("  ✓ Each motor configured and controlled independently");
    println!("\nNote: In RS485, all motors share the same physical wires.");
    println!("The Modbus protocol ensures only the addressed motor responds.");
    
    Ok(())
}
