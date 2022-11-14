/// An IRQ driven serial port
///
pub mod uart1 {
    use crate::interrupt;
    use crate::hal::{
        pac,
        prelude::*,
        serial::{
            Event,
            Serial,
        }
    };
    use heapless::spsc::{Consumer, Producer, Queue};
    use stm32f0xx_hal::gpio::{
        gpiob,
        Alternate,
        AF0,
    };

    const TX_Q_SIZE: usize = 128;
    const RX_Q_SIZE: usize = 8;
    
    static mut TX_Q_CONSUMER: Option<Consumer<u8, TX_Q_SIZE>> = None;
    static mut TX_Q_PRODUCER: Option<Producer<u8, TX_Q_SIZE>> = None;
    static mut RX_Q_CONSUMER: Option<Consumer<u8, RX_Q_SIZE>> = None;
    static mut RX_Q_PRODUCER: Option<Producer<u8, RX_Q_SIZE>> = None;

    type TxPinType = gpiob::PB6<Alternate<AF0>>;
    type RxPinType = gpiob::PB7<Alternate<AF0>>;
    static mut SERIAL: Option<Serial<pac::USART1, TxPinType, RxPinType>> = None;

    pub struct Uart1Tx {}

    impl core::fmt::Write for Uart1Tx {
        fn write_str(&mut self, s: &str) -> Result<(), core::fmt::Error> {
            for b in s.bytes() {
                write_byte(b);
            }
            Ok(())
        }
    }

    /// Must be called once during application initialization
    pub fn init(mut serial: Serial<pac::USART1, TxPinType, RxPinType>, irq_prio: u8) {
        //let mut nvic = unsafe { cortex_m::Peripherals::steal().NVIC };
        let core = unsafe { pac::CorePeripherals::steal() };        
        let mut nvic = core.NVIC;

        serial.listen(Event::Rxne);

        static mut RX_Q: Queue<u8, RX_Q_SIZE> = Queue::new();
        static mut TX_Q: Queue<u8, TX_Q_SIZE> = Queue::new();

        let (rx_q_producer, rx_q_consumer) = unsafe { RX_Q.split() };
        let (tx_q_producer, tx_q_consumer) = unsafe { TX_Q.split() };
        
        unsafe {
            RX_Q_PRODUCER = Some(rx_q_producer);
            RX_Q_CONSUMER = Some(rx_q_consumer);
            TX_Q_PRODUCER = Some(tx_q_producer);
            TX_Q_CONSUMER = Some(tx_q_consumer);
            SERIAL = Some(serial);
        
            nvic.set_priority(pac::Interrupt::USART1, irq_prio);
            pac::NVIC::unmask(pac::Interrupt::USART1);
        }
    }

    #[allow(dead_code)]
    pub fn read_byte() -> Option<u8> {
        let rx_q_consumer = unsafe { RX_Q_CONSUMER.as_mut().unwrap_unchecked() };
        rx_q_consumer.dequeue()
    }

    #[allow(dead_code)]
    pub fn write_byte(b: u8) {
        let tx_q_producer = unsafe { TX_Q_PRODUCER.as_mut().unwrap_unchecked() };
        // Drop the byte if the queue is full
        let _ = tx_q_producer.enqueue(b);
        let serial = unsafe{ SERIAL.as_mut().unwrap_unchecked() };
        serial.listen(Event::Txe);
    }

    #[allow(dead_code)]
    pub fn writer() -> Uart1Tx {
        Uart1Tx {}
    }

    #[interrupt]
    fn USART1() {
        let serial = unsafe{ SERIAL.as_mut().unwrap_unchecked() };
        let rx_q_producer = unsafe { RX_Q_PRODUCER.as_mut().unwrap_unchecked() };
        let tx_q_consumer = unsafe { TX_Q_CONSUMER.as_mut().unwrap_unchecked() };
        let usart1 = unsafe { crate::hal::pac::Peripherals::steal().USART1 };

        // Read any available bytes from the serial port
        match serial.read() {
            Ok(rxbyte) => {
                rx_q_producer.enqueue(rxbyte).ok();
            },
            Err(_) => () 
        };

        // Check if there is room to transmit a byte
        let isr = (*usart1).isr.read();
        if isr.txe().bit_is_set() {
            match tx_q_consumer.dequeue() {
                Some(b) => {
                    // If there's a byte available in the Q, send it
                    serial.write(b).ok().unwrap();
                },
                None => {
                    // If the Q is empty, mask the TXE interrupt. It must be re-enabled when data is written to the queue
                    serial.unlisten(Event::Txe);
                }
            }
        }
    }
    
}