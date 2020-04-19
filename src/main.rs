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
use riscv_rt::entry;

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
            1000,
            8400,
            false,
        ),
        hip_front_right: &PwmServo::new(
            pin!(pins, dig3),
            &pwm1,
            1000,
            8400,
            true,
        ),
        hip_rear_left: &PwmServo::new(
            pin!(pins, dig16),
            &pwm2,
            1000,
            8400,
            true,
        ),
        hip_rear_right: &PwmServo::new(
            pin!(pins, dig17),
            &pwm2,
            1000,
            8400,
            false,
        ),
    };

    allbot.init();

    loop {
        allbot.test(&mut sleep, 2000);
    }
}

struct ALLBOT<'a> {
    hip_front_left: &'a dyn Servo,
    hip_front_right: &'a dyn Servo,
    hip_rear_left: &'a dyn Servo,
    hip_rear_right: &'a dyn Servo,
}

impl<'a> ALLBOT<'a> {
    fn init(&self) {
        self.hip_front_left.write(90.0.degrees());
        self.hip_front_right.write(90.0.degrees());
        self.hip_rear_left.write(90.0.degrees());
        self.hip_rear_right.write(90.0.degrees());
    }

    fn test(&self, sleep: &mut Sleep, speed: u32) {
        animate(
            &[
                Move::new(self.hip_front_left, 45.0.degrees()),
                Move::new(self.hip_front_right, 45.0.degrees()),
                Move::new(self.hip_rear_left, 45.0.degrees()),
                Move::new(self.hip_rear_right, 45.0.degrees()),
            ],
            speed,
            sleep,
        );

        animate(
            &[
                Move::new(self.hip_front_left, 90.0.degrees()),
                Move::new(self.hip_front_right, 90.0.degrees()),
                Move::new(self.hip_rear_left, 90.0.degrees()),
                Move::new(self.hip_rear_right, 90.0.degrees()),
            ],
            speed,
            sleep,
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
