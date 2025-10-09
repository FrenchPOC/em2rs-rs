# em2rs - EM2RS Stepper Motor Controller Library

Rust library for controlling EM2RS stepper motor controllers via Modbus RTU. Provides both asynchronous and synchronous APIs with support for multiple motor instances on a single RS485 bus.

## Features

- **Async & Sync APIs**: Choose between tokio-based async or blocking synchronous operations
- **Multiple Motors**: Control multiple motors with different IDs on the same RS485 bus
- **Complete Register Access**: Full access to all EM2RS registers and functions
- **Type-Safe**: Rust enums for directions, commands, and configurations
- **tokio-modbus**: Built on the reliable tokio-modbus RTU transport layer

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
em2rs = "0.1"
tokio = { version = "1", features = ["full"] }
tokio-modbus = { version = "0.16.5", default-features = false, features = ["rtu", "rtu-sync"] }
tokio-serial = "5.4"
```

## Quick Start

### Async Example

```rust
use em2rs::{Em2rsClient, StepperConfig, Direction, PathConfig};
use tokio_modbus::prelude::*;
use tokio_serial::SerialStream;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Open serial port
    let builder = tokio_serial::new("/dev/ttyUSB0", 9600);
    let port = SerialStream::open(&builder)?;
    
    // Create Modbus RTU context for slave ID 1
    let ctx = rtu::attach_slave(port, Slave::from(1));
    
    // Configure motor
    let config = StepperConfig::new(1, 10000)
        .with_phase_current(2.0)
        .with_direction(Direction::Clockwise);
    
    // Create and initialize client
    let mut motor = Em2rsClient::new(ctx, config);
    motor.init().await?;
    
    // Configure and start a path
    let mut path = PathConfig::new(0)?;
    path.position = 5000;
    path.velocity = 300;
    motor.apply_path_config(&path).await?;
    motor.start_path(0).await?;
    
    // Wait for completion
    while !motor.is_path_completed().await? {
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    }
    
    Ok(())
}
```

### Sync Example

```rust
use em2rs::{Em2rsSyncClient, StepperConfig, PathConfig};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let runtime = tokio::runtime::Runtime::new()?;
    
    let ctx = runtime.block_on(async {
        let builder = tokio_serial::new("/dev/ttyUSB0", 9600);
        let port = tokio_serial::SerialStream::open(&builder)?;
        Ok::<_, Box<dyn std::error::Error>>(
            tokio_modbus::prelude::rtu::attach_slave(port, 
                tokio_modbus::prelude::Slave::from(1))
        )
    })?;
    
    let config = StepperConfig::new(1, 10000);
    let mut motor = Em2rsSyncClient::new(ctx, config);
    motor.init()?;
    
    // All operations are blocking
    let mut path = PathConfig::new(0)?;
    path.position = 5000;
    motor.apply_path_config(&path)?;
    motor.start_path(0)?;
    
    Ok(())
}
```

### Multiple Motors

```rust
// Each motor needs its own context with the appropriate slave ID
let port = SerialStream::open(&builder)?;
let ctx = rtu::attach_slave(port, Slave::from(1));

let motor1 = Em2rsClient::new(ctx, StepperConfig::new(1, 10000));

// For additional motors, dynamically switch slave IDs or use separate ports
// The EM2RS controller identifies itself by the slave ID in each Modbus request
```

## Core Operations

### Initialization
- `init()` - Initialize motor with pulse per rev, direction, current, inductance

### Motion Control
- `start_path(id)` - Execute a configured path (0-8)
- `start_homing()` - Run homing sequence
- `stop_motor()` - Quick stop
- `jog_motor(direction)` - Jog in specified direction
- `manual_zero()` - Set current position as zero

### Configuration
- `apply_path_config(config)` - Configure path with position, velocity, acceleration
- `apply_homing_config(config)` - Configure homing parameters
- `set_peak_current(current)` - Set motor phase current
- `set_soft_limit_max/min(pos)` - Set software position limits
- `configure_input(no, function, nc)` - Configure digital inputs

### Status & Monitoring
- `get_motion_status()` - Get motion status flags
- `is_path_completed()` - Check if path finished
- `is_homing_completed()` - Check if homing finished
- `get_current_alarm()` - Read alarm flags
- `get_version()` - Get firmware version

### Persistence
- `save_param_eeprom()` - Save parameters to EEPROM
- `param_reset()` - Reset parameters (except motor params)
- `factory_reset()` - Full factory reset

## Examples

Run examples with:
```bash
cargo run --example async_example
cargo run --example sync_example
cargo run --example multiple_motors
```

## Reference

Based on the C library by Axel Chabot for EM2RS stepper motor controllers. See `datasheet/EM2RS_Series_User_Manual_V1.5.pdf` for hardware details.

## License

MIT
