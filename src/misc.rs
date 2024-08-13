use heapless::spsc::Queue;

use crate::constants::{SLIP_END, SLIP_ESC, SLIP_ESC_END, SLIP_ESC_ESC, SLIP_ESC_START, SLIP_START};

pub fn enqueue_ditch_oldest(queue: &mut Queue<u8, 1024>, b: u8) {
    match queue.enqueue(SLIP_START) {
        Ok(_) => {},
        Err(b) => {
            queue.dequeue(); // Drop the oldest
            queue.enqueue(b).unwrap();
        }
    }
}

pub fn slip_enqueue(queue: &mut Queue<u8, 1024>, b: u8) {
    match b {
        SLIP_START => {
            enqueue_ditch_oldest(queue, SLIP_ESC);
            enqueue_ditch_oldest(queue, SLIP_ESC_START);
        },
        SLIP_ESC => {
            enqueue_ditch_oldest(queue, SLIP_ESC);
            enqueue_ditch_oldest(queue, SLIP_ESC_ESC);
        },
        SLIP_END => {
            enqueue_ditch_oldest(queue, SLIP_ESC);
            enqueue_ditch_oldest(queue, SLIP_ESC_END);
        },
        _ => {
            enqueue_ditch_oldest(queue, b);
        }
    }
}