#![no_std]

use core::marker::PhantomData;

#[macro_use]
pub mod macros;

pub mod i2c;

pub struct PeripheralsIndev {
    pub i2c: I2C,
}

static mut _TAKEN: bool = false;

impl PeripheralsIndev {
    pub fn take() -> Option<Self> {
        critical_section::with(|_| {
            if unsafe { _TAKEN } {
                None
            } else {
                unsafe {
                    _TAKEN = true;
                }
                Some(PeripheralsIndev {
                    i2c: I2C {
                        _marker: PhantomData,
                    },
                })
            }
        })
    }
    pub unsafe fn steal() -> PeripheralsIndev {
        PeripheralsIndev {
            i2c: I2C {
                _marker: PhantomData,
            },
        }
    }
}

pub struct I2C {
    _marker: PhantomData<*const ()>,
}

impl I2C {
    pub fn write_i2c(&self, val: usize) {
        unsafe {
            i2c::write_i2c::Bits::write(val);
        }
    }
}
