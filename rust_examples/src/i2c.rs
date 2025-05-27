#[allow(unused_imports)]
use core::arch::asm;

pub mod write_i2c {
    pub struct Bits;

    impl Bits {
        read_csr_as_usize!(0x060);
        write_csr_as!(0x060);
        set!(0x060);
        clear!(0x060);
    }
}