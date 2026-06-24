use std::cell::UnsafeCell;
use std::mem;
use std::mem::MaybeUninit;
use std::sync::Arc;
use simhark::{BallState, WorldState};
use crate::Commands;

#[derive(Debug, Clone)]
pub(super) struct RootData {
    inner: Arc<UnsafeCell<[Commands]>>,
}

pub struct Data {
    inner: Arc<UnsafeCell<[Commands]>>,
}


impl RootData {
    pub fn new(num: usize) -> Self {
        let data: Arc<[MaybeUninit<Commands>]> = Arc::new_zeroed_slice(num);
        let dat = Arc::into_raw(data) as *mut [MaybeUninit<Commands>];
        let d = unsafe { &mut *dat };

        for i in 0..num {
            d[i].write([None; 16]);
        }

        let data = unsafe { Arc::from_raw(dat as *const [MaybeUninit<Commands>]) };
        let data = unsafe { data.assume_init() };

        Self {
            inner: unsafe { mem::transmute::<Arc<[Commands]>, Arc<UnsafeCell<[Commands]>>>(data) }
        }
    }

    pub fn set_from(&mut self, data: &[Commands]) {
        unsafe { &mut *self.inner.get() }.copy_from_slice(data);
    }

    pub fn get(&self, idx: usize) -> Commands {
        // This is safe enough, considering that one can only update data via the root data
        unsafe {
            (&*self.inner.get())[idx]
        }
    }

    pub fn read(&self) -> Data {
        Data {
            inner: self.inner.clone(),
        }
    }
}

impl Data {
    pub fn get(&self, idx: usize) -> Commands {
        // This is safe enough, considering that one can only update data via the root data
        unsafe {
            (&*self.inner.get())[idx]
        }
    }
}


pub fn empty_world_state(id: usize) -> WorldState {
    WorldState {
        world_id: id,
        sim_time: 0.0,
        frame: 0,
        ball: BallState {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
        },
        blue_robots: Vec::new(),
        yellow_robots: Vec::new(),
        goal_blue: false,
        goal_yellow: false,
    }
}
