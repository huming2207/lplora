use core::ptr;

pub fn read_uid() -> [u8; 20] {
    let uid_1_ptr = 0x1fff7590 as *const u32;
    let uid_2_ptr = 0x1fff7594 as *const u32;
    let uid_3_ptr = 0x1fff7598 as *const u32;
    let uid64_ptr = 0x1fff7580 as *const u64;

    let uid_1: [u8; 4] = unsafe { uid_1_ptr.read_volatile().to_le_bytes() };
    let uid_2: [u8; 4] = unsafe { uid_2_ptr.read_volatile().to_le_bytes() };
    let uid_3: [u8; 4] = unsafe { uid_3_ptr.read_volatile().to_le_bytes() };
    let uid64: [u8; 8] = unsafe { uid64_ptr.read_volatile().to_le_bytes() };
    let mut ret: [u8; 20] = [0; 20];

    unsafe {
        let ret_ptr = ret.as_mut_ptr();
        ptr::copy_nonoverlapping(uid_1.as_ptr(), ret_ptr, 4);
        ptr::copy_nonoverlapping(uid_2.as_ptr(), ret_ptr.add(4), 4);
        ptr::copy_nonoverlapping(uid_3.as_ptr(), ret_ptr.add(8), 4);
        ptr::copy_nonoverlapping(uid64.as_ptr(), ret_ptr.add(12), 8);
    }

    ret
}
