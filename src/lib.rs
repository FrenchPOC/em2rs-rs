//! EM2RS Stepper Motor Controller Library
//!
//! This library provides both async and sync interfaces for controlling
//! EM2RS stepper motor controllers via Modbus RTU.
//!
//! # Features
//! - Async API using tokio-modbus
//! - Synchronous wrapper for blocking contexts
//! - Support for multiple motor instances on the same bus
//! - Complete register access and high-level operations
//!
//! # Examples
//!
//! ## Async Usage
//! ```no_run
//! use em2rs::{Em2rsClient, StepperConfig, Direction};
//! use tokio_modbus::prelude::*;
//! use tokio_serial::SerialStream;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     // Initialize serial port
//!     let builder = tokio_serial::new("/dev/ttyUSB0", 9600);
//!     let port = SerialStream::open(&builder)?;
//!     
//!     // Create Modbus RTU context
//!     let ctx = rtu::attach_slave(port, Slave::from(1));
//!     
//!     // Create stepper configuration
//!     let config = StepperConfig::new(1, 10000)
//!         .with_phase_current(2.0)
//!         .with_direction(Direction::Clockwise);
//!     
//!     // Create and initialize client
//!     let mut client = Em2rsClient::new(ctx, config);
//!     client.init().await?;
//!     
//!     Ok(())
//! }
//! ```
//!
//! ## Sync Usage
//! ```no_run
//! use em2rs::{Em2rsSyncClient, StepperConfig};
//!
//! fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     // Setup similar to async, but use Em2rsSyncClient
//!     // All methods are blocking
//!     Ok(())
//! }
//! ```

pub mod registers;
pub mod types;
pub mod client;
pub mod sync;

pub use client::Em2rsClient;
pub use sync::Em2rsSyncClient;
pub use types::*;
