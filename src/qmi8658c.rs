use embedded_hal::blocking::i2c::{Write, WriteRead};

const QMI8658C_ADDRESS: u8 = 0x12;
const QMI8658C_ACC_GYRO_OUTPUT: u8 = 0x01;

pub struct QMI8658C<I2C> {
    i2c: I2C,
}

use core::fmt::{self, Display};

impl Display for AccelGyroData {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Accel: ({:.2}, {:.2}, {:.2}), Gyro: ({:.2}, {:.2}, {:.2})",
            self.accel_x,
            self.accel_y,
            self.accel_z,
            self.gyro_x,
            self.gyro_y,
            self.gyro_z,
        )
    }
}

/*
use defmt::Format;

impl Format for AccelGyroData {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Accel: ({:.2}, {:.2}, {:.2}), Gyro: ({:.2}, {:.2}, {:.2})",
            self.accel_x,
            self.accel_y,
            self.accel_z,
            self.gyro_x,
            self.gyro_y,
            self.gyro_z,
        );
    }
}
*/
#[derive(Debug, Default)]
pub struct AccelGyroData {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
}

impl<I2C, E> QMI8658C<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        QMI8658C { i2c }
    }

    pub fn init(&mut self) -> Result<(), E> {
        // Perform any necessary initialization for the QMI8658C sensor
        // (e.g., set up registers, modes, or configurations).
        // Use the `self.i2c.write()` and `self.i2c.write_read()` methods as needed.

        Ok(())
    }

    pub fn read_accel_gyro(&mut self) -> Result<AccelGyroData, E> {
        let mut buffer = [0u8; 12];
        self.i2c
            .write_read(QMI8658C_ADDRESS, &[QMI8658C_ACC_GYRO_OUTPUT], &mut buffer)?;

        let accel_x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let accel_y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let accel_z = i16::from_le_bytes([buffer[4], buffer[5]]);
        let gyro_x = i16::from_le_bytes([buffer[6], buffer[7]]);
        let gyro_y = i16::from_le_bytes([buffer[8], buffer[9]]);
        let gyro_z = i16::from_le_bytes([buffer[10], buffer[11]]);

        Ok(AccelGyroData {
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
        })
    }
}

