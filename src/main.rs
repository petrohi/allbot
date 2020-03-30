#![no_std]
#![no_main]

extern crate panic_halt;

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

    let servos: [&dyn Servo; 4] = [
        &PwmServo::new(pin!(pins, dig4), &pwm1, 3277, 6554),
        &PwmServo::new(pin!(pins, dig3), &pwm1, 3277, 6554),
        &PwmServo::new(pin!(pins, dig16), &pwm2, 3277, 6554),
        &PwmServo::new(pin!(pins, dig17), &pwm2, 3277, 6554),
    ];

    loop {
        sleep.delay_ms(1000u32);

        for servo in servos.iter() {
            servo.write(0.0.degrees());
        }

        sleep.delay_ms(1000u32);

        for servo in servos.iter() {
            servo.write(180.0.degrees());
        }
    }
}

struct Degrees(f64);

trait F64Ext {
    fn degrees(self) -> Degrees;
}

impl F64Ext for f64 {
    fn degrees(self) -> Degrees {
        if self > 180.0 || self < 0.0 {
            panic!("Invalid angle");
        }
        Degrees(self)
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
}

trait PwmServoConstructor<'a, TUnknownPin, TPwmPin, TPWM> {
    fn new(
        pin: TUnknownPin,
        pwm: &'a TPWM,
        duty_at_0_degrees: u16,
        duty_at_180_degrees: u16,
    ) -> PwmServo<'a, TPwmPin, TPWM>;
}

fn degrees_to_duty(
    duty_at_0_degrees: u16,
    duty_at_180_degrees: u16,
    degrees: Degrees,
) -> u16 {
    (duty_at_0_degrees as f64
        + ((degrees.0 / 180.0)
            * (duty_at_180_degrees - duty_at_0_degrees) as f64)) as u16
}

fn duty_to_degrees(
    duty_at_0_degrees: u16,
    duty_at_180_degrees: u16,
    duty: u16,
) -> Degrees {
    Degrees(
        ((duty - duty_at_0_degrees) as f64
            / (duty_at_180_degrees - duty_at_0_degrees) as f64)
            * 180.0,
    )
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
    ) -> PwmServo<'_, gpio0::Pin20<IOF1<Invert>>, PWM1> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
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
    ) -> PwmServo<'_, gpio0::Pin19<IOF1<Invert>>, PWM1> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
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
    ) -> PwmServo<'_, gpio0::Pin10<IOF1<Invert>>, PWM2> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
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
    ) -> PwmServo<'_, gpio0::Pin11<IOF1<Invert>>, PWM2> {
        pwm.cfg
            .write(|w| unsafe { w.enalways().bit(true).scale().bits(6) });

        PwmServo {
            _pin: pin.into_inverted_iof1(),
            pwm: pwm,
            duty_at_0_degrees,
            duty_at_180_degrees,
        }
    }
}

impl Servo for PwmServo<'_, gpio0::Pin20<IOF1<Invert>>, PWM1> {
    fn read(&self) -> Degrees {
        duty_to_degrees(
            self.duty_at_0_degrees,
            self.duty_at_180_degrees,
            self.pwm.cmp0.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp0.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
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
            self.pwm.cmp1.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp1.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
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
            self.pwm.cmp0.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp0.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
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
            self.pwm.cmp1.read().value().bits(),
        )
    }

    fn write(&self, degrees: Degrees) -> () {
        self.pwm.cmp1.write(|w| unsafe {
            w.value().bits(degrees_to_duty(
                self.duty_at_0_degrees,
                self.duty_at_180_degrees,
                degrees,
            ))
        });
    }
}
