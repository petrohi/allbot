#![no_std]
#![no_main]

extern crate panic_halt;

use heapless::consts::U8;
use heapless::Vec;
use hifive1::hal::delay::Sleep;
use hifive1::hal::device::DevicePeripherals;
use hifive1::hal::e310x::PWM1;
use hifive1::hal::e310x::PWM2;
use hifive1::hal::gpio::*;
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::pin;
use rand_core::RngCore;
use riscv_rt::entry;
use wyhash::WyRng;

#[entry]
fn main() -> ! {
    let DeviceResources {
        core_peripherals,
        peripherals,
        pins,
        ..
    } = DeviceResources::take().unwrap();
    let DevicePeripherals {
        PRCI: prci,
        AONCLK: aonclk,
        PWM1: pwm1,
        PWM2: pwm2,
        ..
    } = peripherals;

    let clocks = hifive1::clock::configure(prci, aonclk, 208.mhz().into());
    let mut sleep = Sleep::new(core_peripherals.clint.mtimecmp, clocks);

    let allbot = ALLBOT {
        hip_front_left: &PwmServo::new(
            pin!(pins, dig4),
            &pwm1,
            1200,
            8200,
            false,
        ),
        hip_front_right: &PwmServo::new(
            pin!(pins, dig3),
            &pwm1,
            1200,
            8200,
            true,
        ),
        knee_front_left: &PwmServo::new(
            pin!(pins, dig5),
            &pwm1,
            1200,
            8200,
            true,
        ),
        knee_front_right: &PwmServo::new(
            pin!(pins, dig6),
            &pwm1,
            1200,
            8200,
            false,
        ),
        hip_rear_left: &PwmServo::new(
            pin!(pins, dig16),
            &pwm2,
            1200,
            8200,
            true,
        ),
        hip_rear_right: &PwmServo::new(
            pin!(pins, dig17),
            &pwm2,
            1200,
            8200,
            false,
        ),
        knee_rear_left: &PwmServo::new(
            pin!(pins, dig18),
            &pwm2,
            1200,
            8200,
            true,
        ),
        knee_rear_right: &PwmServo::new(
            pin!(pins, dig19),
            &pwm2,
            1200,
            8200,
            false,
        ),
    };

    allbot.init();

    let mut rng = WyRng::default();

    loop {
        let animation = rng.next_u32() % 17;
        let speed = ((rng.next_u32() % 4) + 1) * 125;
        let steps = (rng.next_u32() % 5) + 1;

        match animation {
            0 => allbot.look_right(&mut sleep, speed),
            1 => allbot.look_left(&mut sleep, speed),
            2 => allbot.wave_rear_right(&mut sleep, steps, speed),
            3 => allbot.wave_rear_left(&mut sleep, steps, speed),
            4 => allbot.wave_front_right(&mut sleep, steps, speed),
            5 => allbot.wave_front_left(&mut sleep, steps, speed),
            6 => allbot.scared(&mut sleep, steps, speed),
            7 => allbot.turn_right(&mut sleep, steps, speed),
            8 => allbot.turn_left(&mut sleep, steps, speed),
            9 => allbot.walk_right(&mut sleep, steps, speed),
            10 => allbot.walk_left(&mut sleep, steps, speed),
            11 => allbot.walk_forward(&mut sleep, steps, speed),
            12 => allbot.walk_backward(&mut sleep, steps, speed),
            13 => allbot.lean_right(&mut sleep, speed),
            14 => allbot.lean_left(&mut sleep, speed),
            15 => allbot.lean_forward(&mut sleep, speed),
            16 => allbot.lean_backward(&mut sleep, speed),
            _ => panic!(),
        }

        sleep.delay_ms(speed);
    }
}

struct ALLBOT<'a> {
    hip_front_left: &'a dyn Servo,
    hip_front_right: &'a dyn Servo,
    hip_rear_left: &'a dyn Servo,
    hip_rear_right: &'a dyn Servo,
    knee_front_left: &'a dyn Servo,
    knee_front_right: &'a dyn Servo,
    knee_rear_left: &'a dyn Servo,
    knee_rear_right: &'a dyn Servo,
}

fn wave(
    knee: &dyn Servo,
    hip: &dyn Servo,
    sleep: &mut Sleep,
    waves: u32,
    speed: u32,
) {
    animate(&[Move::new(knee, 180.0.degrees())], speed, sleep);

    for _ in 0..waves {
        animate(&[Move::new(hip, 0.0.degrees())], speed, sleep);
        animate(&[Move::new(hip, 65.0.degrees())], speed, sleep);
        animate(&[Move::new(hip, 0.0.degrees())], speed, sleep);
        animate(&[Move::new(hip, 45.0.degrees())], speed, sleep);
    }

    animate(&[Move::new(knee, 45.0.degrees())], speed, sleep);
}

fn look(
    hip_rear_left: &dyn Servo,
    hip_rear_right: &dyn Servo,
    hip_front_left: &dyn Servo,
    hip_front_right: &dyn Servo,
    sleep: &mut Sleep,
    speed: u32,
) {
    animate(
        &[
            Move::new(hip_rear_left, 80.0.degrees()),
            Move::new(hip_rear_right, 10.0.degrees()),
            Move::new(hip_front_left, 10.0.degrees()),
            Move::new(hip_front_right, 80.0.degrees()),
        ],
        speed,
        sleep,
    );
    sleep.delay_ms(speed / 2);

    animate(
        &[
            Move::new(hip_rear_left, 45.0.degrees()),
            Move::new(hip_rear_right, 45.0.degrees()),
            Move::new(hip_front_left, 45.0.degrees()),
            Move::new(hip_front_right, 45.0.degrees()),
        ],
        speed,
        sleep,
    );
}

fn lean(knee1: &dyn Servo, knee2: &dyn Servo, sleep: &mut Sleep, speed: u32) {
    animate(
        &[
            Move::new(knee1, 90.0.degrees()),
            Move::new(knee2, 90.0.degrees()),
        ],
        speed,
        sleep,
    );
    sleep.delay_ms(speed / 2);
    animate(
        &[
            Move::new(knee1, 45.0.degrees()),
            Move::new(knee2, 45.0.degrees()),
        ],
        speed,
        sleep,
    );
}

fn walk(
    knees1: (&dyn Servo, &dyn Servo),
    knees2: (&dyn Servo, &dyn Servo),
    hips1: (&dyn Servo, &dyn Servo),
    hips2: (&dyn Servo, &dyn Servo),
    sleep: &mut Sleep,
    steps: u32,
    speed: u32,
) {
    for _ in 0..steps {
        animate(
            &[
                Move::new(knees1.0, 80.0.degrees()),
                Move::new(knees1.1, 80.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(hips1.0, 80.0.degrees()),
                Move::new(hips1.1, 20.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(knees1.0, 30.0.degrees()),
                Move::new(knees1.1, 30.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(hips1.0, 45.0.degrees()),
                Move::new(hips1.1, 45.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(knees1.0, 45.0.degrees()),
                Move::new(knees1.1, 45.0.degrees()),
            ],
            speed,
            sleep,
        );

        animate(
            &[
                Move::new(knees2.0, 80.0.degrees()),
                Move::new(knees2.1, 80.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(hips2.0, 20.0.degrees()),
                Move::new(hips2.1, 80.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(knees2.0, 30.0.degrees()),
                Move::new(knees2.1, 30.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(hips2.0, 45.0.degrees()),
                Move::new(hips2.1, 45.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(knees2.0, 45.0.degrees()),
                Move::new(knees2.1, 45.0.degrees()),
            ],
            speed,
            sleep,
        );
    }
}

fn turn(
    knees1: (&dyn Servo, &dyn Servo),
    knees2: (&dyn Servo, &dyn Servo),
    hips1: (&dyn Servo, &dyn Servo),
    hips2: (&dyn Servo, &dyn Servo),
    sleep: &mut Sleep,
    steps: u32,
    left: bool,
    speed: u32,
) {
    let angles = if left {
        (80.0.degrees(), 20.0.degrees())
    } else {
        (20.0.degrees(), 80.0.degrees())
    };

    for _ in 0..steps {
        animate(
            &[
                Move::new(knees1.0, 80.0.degrees()),
                Move::new(knees1.1, 80.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[Move::new(hips1.0, angles.0), Move::new(hips1.1, angles.0)],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(knees1.0, 30.0.degrees()),
                Move::new(knees1.1, 30.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(hips1.0, 45.0.degrees()),
                Move::new(hips1.1, 45.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(knees1.0, 45.0.degrees()),
                Move::new(knees1.1, 45.0.degrees()),
            ],
            speed,
            sleep,
        );

        animate(
            &[
                Move::new(knees2.0, 80.0.degrees()),
                Move::new(knees2.1, 80.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[Move::new(hips2.0, angles.1), Move::new(hips2.1, angles.1)],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(knees2.0, 30.0.degrees()),
                Move::new(knees2.1, 30.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(hips2.0, 45.0.degrees()),
                Move::new(hips2.1, 45.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(knees2.0, 45.0.degrees()),
                Move::new(knees2.1, 45.0.degrees()),
            ],
            speed,
            sleep,
        );
    }
}

impl<'a> ALLBOT<'a> {
    fn init(&self) {
        self.hip_front_left.write(45.0.degrees());
        self.hip_front_right.write(45.0.degrees());
        self.hip_rear_left.write(45.0.degrees());
        self.hip_rear_right.write(45.0.degrees());
        self.knee_front_left.write(45.0.degrees());
        self.knee_front_right.write(45.0.degrees());
        self.knee_rear_left.write(45.0.degrees());
        self.knee_rear_right.write(45.0.degrees());
    }

    fn wave_front_right(&self, sleep: &mut Sleep, waves: u32, speed: u32) {
        wave(
            self.knee_front_right,
            self.hip_front_right,
            sleep,
            waves,
            speed,
        );
    }

    fn wave_front_left(&self, sleep: &mut Sleep, waves: u32, speed: u32) {
        wave(
            self.knee_front_left,
            self.hip_front_left,
            sleep,
            waves,
            speed,
        );
    }

    fn wave_rear_right(&self, sleep: &mut Sleep, waves: u32, speed: u32) {
        wave(
            self.knee_rear_right,
            self.hip_rear_right,
            sleep,
            waves,
            speed,
        );
    }

    fn wave_rear_left(&self, sleep: &mut Sleep, waves: u32, speed: u32) {
        wave(self.knee_rear_left, self.hip_rear_left, sleep, waves, speed);
    }

    fn scared(&self, sleep: &mut Sleep, shakes: u32, speed: u32) {
        animate(
            &[
                Move::new(self.knee_front_right, 0.0.degrees()),
                Move::new(self.knee_rear_right, 0.0.degrees()),
                Move::new(self.knee_front_left, 0.0.degrees()),
                Move::new(self.knee_rear_left, 0.0.degrees()),
            ],
            speed,
            sleep,
        );
        for _ in 0..shakes {
            animate(
                &[
                    Move::new(self.hip_rear_right, 80.0.degrees()),
                    Move::new(self.hip_rear_left, 10.0.degrees()),
                    Move::new(self.hip_front_right, 10.0.degrees()),
                    Move::new(self.hip_front_left, 80.0.degrees()),
                ],
                speed * 2,
                sleep,
            );
            animate(
                &[
                    Move::new(self.hip_rear_right, 10.0.degrees()),
                    Move::new(self.hip_rear_left, 80.0.degrees()),
                    Move::new(self.hip_front_right, 80.0.degrees()),
                    Move::new(self.hip_front_left, 10.0.degrees()),
                ],
                speed * 2,
                sleep,
            );
        }
        animate(
            &[
                Move::new(self.hip_rear_right, 45.0.degrees()),
                Move::new(self.hip_rear_left, 45.0.degrees()),
                Move::new(self.hip_front_right, 45.0.degrees()),
                Move::new(self.hip_front_left, 45.0.degrees()),
            ],
            speed,
            sleep,
        );
        animate(
            &[
                Move::new(self.knee_front_right, 45.0.degrees()),
                Move::new(self.knee_rear_right, 45.0.degrees()),
                Move::new(self.knee_front_left, 45.0.degrees()),
                Move::new(self.knee_rear_left, 45.0.degrees()),
            ],
            speed,
            sleep,
        );
    }

    fn look_left(&self, sleep: &mut Sleep, speed: u32) {
        look(
            self.hip_rear_left,
            self.hip_rear_right,
            self.hip_front_left,
            self.hip_front_right,
            sleep,
            speed,
        );
    }

    fn look_right(&self, sleep: &mut Sleep, speed: u32) {
        look(
            self.hip_rear_right,
            self.hip_rear_left,
            self.hip_front_right,
            self.hip_front_left,
            sleep,
            speed,
        );
    }

    fn lean_right(&self, sleep: &mut Sleep, speed: u32) {
        lean(self.knee_front_right, self.knee_rear_right, sleep, speed);
    }

    fn lean_left(&self, sleep: &mut Sleep, speed: u32) {
        lean(self.knee_front_left, self.knee_rear_left, sleep, speed);
    }

    fn lean_forward(&self, sleep: &mut Sleep, speed: u32) {
        lean(self.knee_front_left, self.knee_front_right, sleep, speed);
    }

    fn lean_backward(&self, sleep: &mut Sleep, speed: u32) {
        lean(self.knee_rear_left, self.knee_rear_right, sleep, speed);
    }

    fn walk_forward(&self, sleep: &mut Sleep, steps: u32, speed: u32) {
        walk(
            (self.knee_rear_right, self.knee_front_left),
            (self.knee_rear_left, self.knee_front_right),
            (self.hip_rear_right, self.hip_front_left),
            (self.hip_front_right, self.hip_rear_left),
            sleep,
            steps,
            speed,
        );
    }

    fn walk_backward(&self, sleep: &mut Sleep, steps: u32, speed: u32) {
        walk(
            (self.knee_rear_right, self.knee_front_left),
            (self.knee_rear_left, self.knee_front_right),
            (self.hip_front_left, self.hip_rear_right),
            (self.hip_rear_left, self.hip_front_right),
            sleep,
            steps,
            speed,
        );
    }

    fn walk_left(&self, sleep: &mut Sleep, steps: u32, speed: u32) {
        walk(
            (self.knee_rear_right, self.knee_front_left),
            (self.knee_rear_left, self.knee_front_right),
            (self.hip_front_left, self.hip_rear_right),
            (self.hip_front_right, self.hip_rear_left),
            sleep,
            steps,
            speed,
        );
    }

    fn walk_right(&self, sleep: &mut Sleep, steps: u32, speed: u32) {
        walk(
            (self.knee_rear_right, self.knee_front_left),
            (self.knee_rear_left, self.knee_front_right),
            (self.hip_rear_right, self.hip_front_left),
            (self.hip_rear_left, self.hip_front_right),
            sleep,
            steps,
            speed,
        );
    }

    fn turn_left(&self, sleep: &mut Sleep, steps: u32, speed: u32) {
        turn(
            (self.knee_rear_right, self.knee_front_left),
            (self.knee_rear_left, self.knee_front_right),
            (self.hip_front_left, self.hip_rear_right),
            (self.hip_front_right, self.hip_rear_left),
            sleep,
            steps,
            true,
            speed,
        );
    }

    fn turn_right(&self, sleep: &mut Sleep, steps: u32, speed: u32) {
        turn(
            (self.knee_rear_right, self.knee_front_left),
            (self.knee_rear_left, self.knee_front_right),
            (self.hip_front_left, self.hip_rear_right),
            (self.hip_front_right, self.hip_rear_left),
            sleep,
            steps,
            false,
            speed,
        );
    }
}

struct Move<'a> {
    servo: &'a dyn Servo,
    desired: Degrees,
}

impl<'a> Move<'a> {
    fn new(servo: &'a dyn Servo, desired: Degrees) -> Move<'a> {
        Move { servo, desired }
    }
}

fn animate(moves: &[Move], speed: u32, sleep: &mut Sleep) -> () {
    const STEP_SPEED: u32 = 20;
    let step_num = speed / STEP_SPEED;

    let mut deltas: Vec<_, U8> = Vec::new();

    for m in moves {
        let current = m.servo.read();
        let delta = (m.desired.0 - current.0) / step_num as f64;

        deltas.push(delta).unwrap();
    }
    for _ in 0..(speed / STEP_SPEED) {
        for i in 0..moves.len() {
            let m = &moves[i];
            let delta = deltas[i];

            let new_degrees = m.servo.read().0 + delta;
            m.servo.write(new_degrees.degrees());
        }

        sleep.delay_ms(STEP_SPEED);
    }
}

#[derive(Clone, Copy)]
struct Degrees(f64);

trait F64Ext {
    fn degrees(self) -> Degrees;
}

impl F64Ext for f64 {
    fn degrees(self) -> Degrees {
        if self > 180.0 {
            Degrees(180.0)
        } else if self < 0.0 {
            Degrees(0.0)
        } else {
            Degrees(self)
        }
    }
}

trait Servo {
    fn read(&self) -> Degrees;
    fn write(&self, degrees: Degrees) -> ();
}

struct PwmServo<'a, TPin, TPWM> {
    _pin: TPin,
    pwm: &'a TPWM,
    duty_at_0_degrees: u16,
    duty_at_180_degrees: u16,
    inverted: bool,
}

trait PwmServoConstructor<'a, TUnknownPin, TPwmPin, TPWM> {
    fn new(
        pin: TUnknownPin,
        pwm: &'a TPWM,
        duty_at_0_degrees: u16,
        duty_at_180_degrees: u16,
        inverted: bool,
    ) -> PwmServo<'a, TPwmPin, TPWM>;
}

fn degrees_to_duty(
    duty_at_0_degrees: u16,
    duty_at_180_degrees: u16,
    inverted: bool,
    degrees: Degrees,
) -> u16 {
    let norm_degrees = if inverted {
        180.0 - degrees.0
    } else {
        degrees.0
    };
    (duty_at_0_degrees as f64
        + ((norm_degrees / 180.0)
            * (duty_at_180_degrees - duty_at_0_degrees) as f64)) as u16
}

fn duty_to_degrees(
    duty_at_0_degrees: u16,
    duty_at_180_degrees: u16,
    inverted: bool,
    duty: u16,
) -> Degrees {
    let norm_degrees = ((duty - duty_at_0_degrees) as f64
        / (duty_at_180_degrees - duty_at_0_degrees) as f64)
        * 180.0;
    if inverted {
        Degrees(180.0 - norm_degrees)
    } else {
        Degrees(norm_degrees)
    }
}

impl
    PwmServoConstructor<
        '_,
        gpio0::Pin20<Unknown>,
        gpio0::Pin20<IOF1<Invert>>,
        PWM1,
    > for PwmServo<'_, gpio0::Pin20<IOF1<Invert>>, PWM1>
{
    fn new(
        pin: gpio0::Pin20<Unknown>,
        pwm: &PWM1,
        duty_at_0_degrees: u16,
        duty_at_180_degrees: u16,
        inverted: bool,
    ) -> PwmServo<'_, gpio0::Pin20<IOF1<Invert>>, PWM1> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
            inverted,
        }
    }
}

impl
    PwmServoConstructor<
        '_,
        gpio0::Pin21<Unknown>,
        gpio0::Pin21<IOF1<Invert>>,
        PWM1,
    > for PwmServo<'_, gpio0::Pin20<IOF1<Invert>>, PWM1>
{
    fn new(
        pin: gpio0::Pin21<Unknown>,
        pwm: &PWM1,
        duty_at_0_degrees: u16,
        duty_at_180_degrees: u16,
        inverted: bool,
    ) -> PwmServo<'_, gpio0::Pin21<IOF1<Invert>>, PWM1> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
            inverted,
        }
    }
}

impl
    PwmServoConstructor<
        '_,
        gpio0::Pin22<Unknown>,
        gpio0::Pin22<IOF1<Invert>>,
        PWM1,
    > for PwmServo<'_, gpio0::Pin22<IOF1<Invert>>, PWM1>
{
    fn new(
        pin: gpio0::Pin22<Unknown>,
        pwm: &PWM1,
        duty_at_0_degrees: u16,
        duty_at_180_degrees: u16,
        inverted: bool,
    ) -> PwmServo<'_, gpio0::Pin22<IOF1<Invert>>, PWM1> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
            inverted,
        }
    }
}

impl
    PwmServoConstructor<
        '_,
        gpio0::Pin19<Unknown>,
        gpio0::Pin19<IOF1<Invert>>,
        PWM1,
    > for PwmServo<'_, gpio0::Pin19<IOF1<Invert>>, PWM1>
{
    fn new(
        pin: gpio0::Pin19<Unknown>,
        pwm: &PWM1,
        duty_at_0_degrees: u16,
        duty_at_180_degrees: u16,
        inverted: bool,
    ) -> PwmServo<'_, gpio0::Pin19<IOF1<Invert>>, PWM1> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
            inverted,
        }
    }
}

impl
    PwmServoConstructor<
        '_,
        gpio0::Pin10<Unknown>,
        gpio0::Pin10<IOF1<Invert>>,
        PWM2,
    > for PwmServo<'_, gpio0::Pin10<IOF1<Invert>>, PWM2>
{
    fn new(
        pin: gpio0::Pin10<Unknown>,
        pwm: &PWM2,
        duty_at_0_degrees: u16,
        duty_at_180_degrees: u16,
        inverted: bool,
    ) -> PwmServo<'_, gpio0::Pin10<IOF1<Invert>>, PWM2> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
            inverted,
        }
    }
}

impl
    PwmServoConstructor<
        '_,
        gpio0::Pin11<Unknown>,
        gpio0::Pin11<IOF1<Invert>>,
        PWM2,
    > for PwmServo<'_, gpio0::Pin11<IOF1<Invert>>, PWM2>
{
    fn new(
        pin: gpio0::Pin11<Unknown>,
        pwm: &PWM2,
        duty_at_0_degrees: u16,
        duty_at_180_degrees: u16,
        inverted: bool,
    ) -> PwmServo<'_, gpio0::Pin11<IOF1<Invert>>, PWM2> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
            inverted,
        }
    }
}

impl
    PwmServoConstructor<
        '_,
        gpio0::Pin12<Unknown>,
        gpio0::Pin12<IOF1<Invert>>,
        PWM2,
    > for PwmServo<'_, gpio0::Pin12<IOF1<Invert>>, PWM2>
{
    fn new(
        pin: gpio0::Pin12<Unknown>,
        pwm: &PWM2,
        duty_at_0_degrees: u16,
        duty_at_180_degrees: u16,
        inverted: bool,
    ) -> PwmServo<'_, gpio0::Pin12<IOF1<Invert>>, PWM2> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
            inverted,
        }
    }
}

impl
    PwmServoConstructor<
        '_,
        gpio0::Pin13<Unknown>,
        gpio0::Pin13<IOF1<Invert>>,
        PWM2,
    > for PwmServo<'_, gpio0::Pin12<IOF1<Invert>>, PWM2>
{
    fn new(
        pin: gpio0::Pin13<Unknown>,
        pwm: &PWM2,
        duty_at_0_degrees: u16,
        duty_at_180_degrees: u16,
        inverted: bool,
    ) -> PwmServo<'_, gpio0::Pin13<IOF1<Invert>>, PWM2> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
            inverted,
        }
    }
}

impl Servo for PwmServo<'_, gpio0::Pin20<IOF1<Invert>>, PWM1> {
    fn read(&self) -> Degrees {
        duty_to_degrees(
            self.duty_at_0_degrees,
            self.duty_at_180_degrees,
            self.inverted,
            self.pwm.cmp0.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp0.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
                self.inverted,
                degrees,
            ))
        });
    }
}

impl Servo for PwmServo<'_, gpio0::Pin19<IOF1<Invert>>, PWM1> {
    fn read(&self) -> Degrees {
        duty_to_degrees(
            self.duty_at_0_degrees,
            self.duty_at_180_degrees,
            self.inverted,
            self.pwm.cmp1.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp1.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
                self.inverted,
                degrees,
            ))
        });
    }
}

impl Servo for PwmServo<'_, gpio0::Pin21<IOF1<Invert>>, PWM1> {
    fn read(&self) -> Degrees {
        duty_to_degrees(
            self.duty_at_0_degrees,
            self.duty_at_180_degrees,
            self.inverted,
            self.pwm.cmp2.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp2.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
                self.inverted,
                degrees,
            ))
        });
    }
}

impl Servo for PwmServo<'_, gpio0::Pin22<IOF1<Invert>>, PWM1> {
    fn read(&self) -> Degrees {
        duty_to_degrees(
            self.duty_at_0_degrees,
            self.duty_at_180_degrees,
            self.inverted,
            self.pwm.cmp3.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp3.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
                self.inverted,
                degrees,
            ))
        });
    }
}

impl Servo for PwmServo<'_, gpio0::Pin10<IOF1<Invert>>, PWM2> {
    fn read(&self) -> Degrees {
        duty_to_degrees(
            self.duty_at_0_degrees,
            self.duty_at_180_degrees,
            self.inverted,
            self.pwm.cmp0.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp0.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
                self.inverted,
                degrees,
            ))
        });
    }
}

impl Servo for PwmServo<'_, gpio0::Pin11<IOF1<Invert>>, PWM2> {
    fn read(&self) -> Degrees {
        duty_to_degrees(
            self.duty_at_0_degrees,
            self.duty_at_180_degrees,
            self.inverted,
            self.pwm.cmp1.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp1.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
                self.inverted,
                degrees,
            ))
        });
    }
}

impl Servo for PwmServo<'_, gpio0::Pin12<IOF1<Invert>>, PWM2> {
    fn read(&self) -> Degrees {
        duty_to_degrees(
            self.duty_at_0_degrees,
            self.duty_at_180_degrees,
            self.inverted,
            self.pwm.cmp2.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp2.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
                self.inverted,
                degrees,
            ))
        });
    }
}

impl Servo for PwmServo<'_, gpio0::Pin13<IOF1<Invert>>, PWM2> {
    fn read(&self) -> Degrees {
        duty_to_degrees(
            self.duty_at_0_degrees,
            self.duty_at_180_degrees,
            self.inverted,
            self.pwm.cmp3.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp3.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
                self.inverted,
                degrees,
            ))
        });
    }
}
