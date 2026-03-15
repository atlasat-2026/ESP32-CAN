#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(version("1.59")), feature(asm))]
#![cfg_attr(
    all(version("1.58"), target_arch = "xtensa"),
    feature(asm_experimental_arch)
)]
#![feature(cfg_version)]
use core::arch::asm;

#[cfg(not(feature = "std"))]
use core::panic::PanicInfo;

#[no_mangle]
pub extern "C" fn add_in_rust(x: i32, y: i32) -> i32 {
    x + y
}

#[no_mangle]
pub extern "C" fn add_in_rust_inline_asm(mut x: i32, y: i32) -> i32 {
    unsafe {
        // more detail available: https://doc.rust-lang.org/beta/unstable-book/library-features/asm.html
        asm!("add {0}, {0}, {1}", inout(reg) x, in(reg) y);
    }
    x
}

#[cfg(not(feature = "std"))]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
