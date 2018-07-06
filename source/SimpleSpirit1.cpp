/*** Mbed Includes ***/
#include "SimpleSpirit1.h"
#include "radio_spi.h"

#define S2LP_GPIO_IRQ			(S2LP_GPIO_3)

static uint16_t last_state;
#define S2LP_STATUS()		((last_state = (uint16_t)refresh_state()) & S2LP_STATE_STATEBITS)

#define XO_ON                   (0x1)

#define BUSYWAIT_UNTIL(cond, millisecs) \
        do { \
            uint32_t start = us_ticker_read(); \
            uint32_t limit = (uint32_t)millisecs*1000U; \
            while (!(cond)) { \
                uint32_t now = us_ticker_read(); \
                if((now - start) > limit) break; \
            } \
        } while(0)

#define st_lib_s2lp_irqs		S2LPIrqs

#define STATE_TIMEOUT           (100)

// betzw: switching force & back from standby is on some devices quite unstable
#define USE_STANDBY_STATE

/*** Class Implementation ***/
/** Static Class Variables **/
SimpleS2LP *SimpleS2LP::_singleton = NULL;

/** Constructor **/
SimpleS2LP::SimpleS2LP(PinName mosi, PinName miso, PinName sclk,
                             PinName irq, PinName cs, PinName sdn,
                             PinName led) :
    		        _spi(mosi, miso, sclk),
    		        _irq(irq),
    		        _chip_select(cs),
    		        _shut_down(sdn),
    		        _led(led),
    		        _current_irq_callback(),
    		        _rx_receiving_timeout()
{
}

/** Init Function **/
void SimpleS2LP::init() {
    /* reset irq disable counter and irq callback & disable irq */
    _nr_of_irq_disables = 0;
    disable_s2lp_irq();

    /* unselect chip */
    chip_unselect();

    /* configure spi */
    _spi.format(8, 0); /* 8-bit, mode = 0, [order = SPI_MSB] only available in mbed3 */
    _spi.frequency(10000000); // 10MHz (i.e. max speed allowed for S2LP)

    /* install irq handler */
    _irq.mode(PullUp);
    _irq.fall(Callback<void()>(this, &SimpleS2LP::IrqHandler));

    /* init cube vars */
    s2lp_on = OFF;
    last_rssi = 0 ; //MGR
    last_sqi = 0 ;  //MGR

    /* set frequencies */
    radio_set_xtal_freq(XTAL_FREQUENCY);
    mgmt_set_freq_base((uint32_t)BASE_FREQUENCY);

    /* restart board */
    enter_shutdown();
    exit_shutdown();

    /* soft core reset */
    cmd_strobe(S2LP_STROBE_SRES);

    /* Configures the S2LP radio part */
    SRadioInit x_radio_init = {
            (uint32_t)BASE_FREQUENCY,
            MODULATION_SELECT,
            DATARATE,
            (uint32_t)FREQ_DEVIATION,
            (uint32_t)BANDWIDTH
    };
    radio_init(&x_radio_init);
    radio_set_pa_level_dbm(0,POWER_DBM);
    radio_set_pa_level_max_index(0);

    /* Configures the S2LP packet handler part*/
    PktBasicInit x_basic_init = {
            PREAMBLE_LENGTH,
            SYNC_LENGTH,
            SYNC_WORD,
            LENGTH_TYPE,
            CONTROL_LENGTH,
            CRC_MODE,
            EN_ADDRESS,
            EN_FEC,
            EN_WHITENING
    };
    pkt_basic_init(&x_basic_init);

    /* Enable the following interrupt sources, routed to GPIO */
    irq_de_init(NULL);
    irq_clear_status();
    irq_set_status(TX_DATA_SENT, S_ENABLE);
    irq_set_status(RX_DATA_READY,S_ENABLE);
    irq_set_status(RX_DATA_DISC, S_ENABLE);
    irq_set_status(VALID_SYNC, S_ENABLE);
    irq_set_status(TX_FIFO_ERROR, S_ENABLE);
    irq_set_status(RX_FIFO_ERROR, S_ENABLE);
#ifndef RX_FIFO_THR_WA
    irq_set_status(TX_FIFO_ALMOST_EMPTY, S_ENABLE);
    irq_set_status(RX_FIFO_ALMOST_FULL, S_ENABLE);
#endif // !RX_FIFO_THR_WA

    /* Configure S2LP */
    radio_persistent_rx(S_ENABLE);
    //qi_set_sqi_threshold(SQI_TH_0);
    //qi_sqi_check(S_ENABLE);
    qi_set_rssi_threshold_dbm(RSSI_RX_THRESHOLD);
    timer_set_rx_timeout_stop_condition(SQI_ABOVE_THRESHOLD);
    timer_set_infinite_rx_timeout();
    //radio_afc_freeze_on_sync(S_ENABLE);
    calibration_rco(S_ENABLE);

    CLEAR_TXBUF();
    CLEAR_RXBUF();
    _s2lp_tx_started = false;
    _is_receiving = false;

    /* Configure the radio to route the IRQ signal to its GPIO 3 */
    SGpioInit x_gpio_init = {
            S2LP_GPIO_IRQ,
            S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
            S2LP_GPIO_DIG_OUT_IRQ
    };
    s2lp_gpio_init(&x_gpio_init);

    /* Setup CSMA/CA */
    SCsmaInit x_csma_init = {
            S_ENABLE,         // enable persistent mode
            CSMA_PERIOD_64TBIT, // Tcca time
            3,                // Lcca length
            5,                // max nr of backoffs (<8)
            1,                // BU counter seed
            8                 // BU prescaler
    };
    csma_ca_init(&x_csma_init);

#ifdef USE_STANDBY_STATE
    /* Puts the S2LP in STANDBY mode (125us -> rx/tx) */
    cmd_strobe(S2LP_STROBE_STANDBY);
#endif // USE_STANDBY_STATE
}

static volatile int tx_fifo_remaining = 0;            // to be used in irq handler
static volatile int tx_buffer_pos = 0;                // to be used in irq handler
static const volatile uint8_t *tx_fifo_buffer = NULL; // to be used in irq handler
int SimpleS2LP::send(const void *payload, unsigned int payload_len, bool use_csma_ca) {
    /* Checks if the payload length is supported */
    if(payload_len > MAX_PACKET_LEN) {
        return TX_ERR;
    }

    disable_s2lp_irq();

    BUSYWAIT_UNTIL(S2LP_STATUS() == S2LP_STATE_RX, STATE_TIMEOUT);
#ifndef NDEBUG
    if((last_state & S2LP_STATE_STATEBITS) != S2LP_STATE_RX) {
        debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, last_state>>1);
    }
#endif

    /* Reset State to Ready */
    set_ready_state();

    cmd_strobe(S2LP_STROBE_FTX); // flush TX FIFO buffer

#ifndef NDEBUG
    debug_if(!(linear_fifo_read_num_elements_tx_fifo() == 0), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif

    pkt_basic_set_payload_length(payload_len); // set desired payload len

    if(use_csma_ca) {
        csma_ca_state(S_ENABLE); // enable CSMA/CA
    }

    /* Init buffer & number of bytes to be send */
    tx_fifo_remaining = payload_len;
    tx_fifo_buffer = (const uint8_t*)payload;

    int8_t fifo_available = S2LP_MAX_FIFO_LEN; // fill-up whole fifo
    int8_t to_send = (tx_fifo_remaining > fifo_available) ? fifo_available : tx_fifo_remaining;

    tx_fifo_remaining -= to_send;

    /* Fill FIFO Buffer */
    if(to_send > 0) {
        spi_write_linear_fifo(to_send, (uint8_t*)&tx_fifo_buffer[0]);
    }

    tx_buffer_pos = to_send;
    _s2lp_tx_started = true;

    enable_s2lp_irq();

    /* Start transmitting */
    cmd_strobe(S2LP_STROBE_TX);

    while(tx_fifo_remaining != 0); // wait until not everything is yet send (evtl. by irq handler)

    BUSYWAIT_UNTIL(!_s2lp_tx_started, STATE_TIMEOUT);
#ifdef HEAVY_DEBUG
    debug("\r\n%s (%d): state=%x, _s2lp_tx_started=%d\r\n", __func__, __LINE__, S2LP_STATUS()>>1, _s2lp_tx_started);
#endif

    if(use_csma_ca) {
        csma_ca_state(S_DISABLE); // disable CSMA/CA
    }

    cmd_strobe(S2LP_STROBE_RX); // Return to RX state

    disable_s2lp_irq();
    if(_s2lp_tx_started) { // in case of state timeout
        _s2lp_tx_started = false;
        enable_s2lp_irq();
        return TX_ERR;
    } else {
        enable_s2lp_irq();
        return TX_DONE;
    }
}

/** Set Ready State **/
void SimpleS2LP::set_ready_state(void) {
    uint16_t state;

    disable_s2lp_irq();

    _s2lp_tx_started = false;
    _is_receiving = false;
    stop_rx_timeout();

    cmd_strobe(S2LP_STROBE_FRX);
    CLEAR_RXBUF();
    CLEAR_TXBUF();

    state = S2LP_STATUS();
    if(state == S2LP_STATE_STANDBY) {
        cmd_strobe(S2LP_STROBE_READY);
    } else if(state == S2LP_STATE_RX) {
        cmd_strobe(S2LP_STROBE_SABORT);
    } else if(state != S2LP_STATE_READY) {
#ifndef NDEBUG
        debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, state>>1);
#endif
    }

    BUSYWAIT_UNTIL((S2LP_STATUS() == S2LP_STATE_READY) && ((last_state & XO_ON) == XO_ON), STATE_TIMEOUT);
    if(last_state != (S2LP_STATE_READY | XO_ON)) {
        error("\r\nS2LP: failed to become ready (%x) => pls. reset!\r\n", last_state);
        enable_s2lp_irq();
        return;
    }

    irq_clear_status();

    enable_s2lp_irq();
}

int SimpleS2LP::off(void) {
    if(s2lp_on == ON) {
        /* Disables the mcu to get IRQ from the S2LP */
        disable_s2lp_irq();

        /* first stop rx/tx */
        set_ready_state();

#ifdef USE_STANDBY_STATE
        /* Puts the S2LP in STANDBY */
        cmd_strobe(S2LP_STROBE_STANDBY);
        BUSYWAIT_UNTIL(S2LP_STATUS() == S2LP_STATE_STANDBY, STATE_TIMEOUT);
        if((last_state & S2LP_STATE_STATEBITS) != S2LP_STATE_STANDBY) {
            error("\r\nS2LP: failed to enter standby (%x)\r\n", last_state>>1);
            return 1;
        }
#endif // USE_STANDBY_STATE

        s2lp_on = OFF;
        _nr_of_irq_disables = 1;
    }
    return 0;
}

int SimpleS2LP::on(void) {
    if(s2lp_on == OFF) {
        set_ready_state();

        /* now we go to Rx */
        cmd_strobe(S2LP_STROBE_RX);

        BUSYWAIT_UNTIL(S2LP_STATUS() == S2LP_STATE_RX, STATE_TIMEOUT);
        if((last_state & S2LP_STATE_STATEBITS) != S2LP_STATE_RX) {
            error("\r\nS2LP: failed to enter rx (%x) => retry\r\n", last_state>>1);
        }

        /* Enables the mcu to get IRQ from the S2LP */
        s2lp_on = ON;
#ifndef NDEBUG
        debug_if(!(_nr_of_irq_disables == 1), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
        enable_s2lp_irq();
    }

#ifndef NDEBUG
    if(S2LP_STATUS() != S2LP_STATE_RX) {
        debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, last_state>>1);
    }
#endif

    return 0;
}

uint8_t SimpleS2LP::refresh_state(void) {
    uint8_t mcstate;

    S2LPSpiReadRegisters(MC_STATE0_ADDR, 1, &mcstate);

    return mcstate;
}

int SimpleS2LP::read(void *buf, unsigned int bufsize)
{
    disable_s2lp_irq();

    /* Checks if the RX buffer is empty */
    if(IS_RXBUF_EMPTY()) {
#ifndef NDEBUG
        debug("\r\nBuffer is empty\r\n");
#endif
        set_ready_state();

        cmd_strobe(S2LP_STROBE_RX);
        BUSYWAIT_UNTIL(S2LP_STATUS() == S2LP_STATE_RX, STATE_TIMEOUT);
        enable_s2lp_irq();
        return 0;
    }

    if(bufsize < s2lp_rx_len) {
        enable_s2lp_irq();

        /* If buf has the correct size */
#ifndef NDEBUG
        debug("\r\nTOO SMALL BUF\r\n");
#endif
        return 0;
    } else {
        /* Copies the packet received */
        memcpy(buf, s2lp_rx_buf, s2lp_rx_len);

        bufsize = s2lp_rx_len;
        CLEAR_RXBUF();

        enable_s2lp_irq();

        return bufsize;
    }

}

/* betzw - TODO: CCA should be reviewed (completely)! */
int SimpleS2LP::channel_clear(void)
{
    float rssi_value;
    /* Local variable used to memorize the S2LP state */
    uint8_t s2lp_state = ON;

    if(s2lp_on == OFF) {
        /* Wakes up the S2LP */
        on();
        s2lp_state = OFF;
    }

#ifndef NDEBUG
    if(S2LP_STATUS() != S2LP_STATE_RX) {
        debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, last_state>>1);
    }
#endif

    disable_s2lp_irq();

    /* Reset State to Ready */
    set_ready_state();

    /* Stores the RSSI value */
    rssi_value = qi_get_rssi_dbm();

    enable_s2lp_irq();

    /* Puts the S2LP in its previous state */
    if(s2lp_state == OFF) {
        off();

#ifndef NDEBUG
#ifdef USE_STANDBY_STATE
        if(S2LP_STATUS() != S2LP_STATE_STANDBY) {
            debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, last_state>>1);
        }
#else // !USE_STANDBY_STATE
        if(S2LP_STATUS() != S2LP_STATE_READY) {
            debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, last_state>>1);
        }
#endif // !USE_STANDBY_STATE
#endif // NDEBUG
    } else { // s2lp_state != OFF
        disable_s2lp_irq();

        set_ready_state();

        cmd_strobe(S2LP_STROBE_RX);
        BUSYWAIT_UNTIL(S2LP_STATUS() == S2LP_STATE_RX, STATE_TIMEOUT);
        if((last_state & S2LP_STATE_STATEBITS) != S2LP_STATE_RX) {
            error("\r\nS2LP: (#2) failed to enter rx (%x) => retry\r\n", last_state>>1);
        }

        enable_s2lp_irq();

#ifndef NDEBUG
        if(S2LP_STATUS() != S2LP_STATE_RX) {
            debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, last_state>>1);
        }
#endif
    }

    /* If the received signal strength is above a certain level the medium is considered busy! */
    /* Compare RSSI value with threshold */
    if(rssi_value < RSSI_RX_THRESHOLD) {
        return 0; // idle
    } else {
        return 1; // busy
    }
}

int SimpleS2LP::get_pending_packet(void)
{
    return !IS_RXBUF_EMPTY();
}

/** S2LP Irq Callback **/
/* betzw - TODO: use threaded interrupt handling when `MBED_CONF_RTOS_PRESENT` is defined (see `atmel-rf-driver`) */
void SimpleS2LP::IrqHandler() {
    st_lib_s2lp_irqs x_irq_status;

    /* get interrupt source from radio */
    irq_get_status(&x_irq_status);

    /* The IRQ_TX_DATA_SENT notifies the packet has been sent. Puts the S2LP in RX */
    if(x_irq_status.IRQ_TX_DATA_SENT) { /* betzw - NOTE: MUST be handled before `IRQ_RX_DATA_READY` for Nanostack integration!
	                                                     Logically, Nanostack only expects the "DONE" after "SUCCESS" (if it gets
	                                                     DONE before SUCCESS, it assumes you're not going to bother to send SUCCESS).
     */
#ifdef DEBUG_IRQ
        uint32_t *tmp = (uint32_t*)&x_irq_status;
        debug_if(!((*tmp) & IRQ_TX_DATA_SENT_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
        debug_if(tx_fifo_remaining != 0, "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif

        if(_s2lp_tx_started) {
            _s2lp_tx_started = false;

            /* call user callback */
            if(_current_irq_callback) {
                _current_irq_callback(TX_DONE);
            }
        }

        /* Disable handling of other TX flags */
        x_irq_status.IRQ_TX_FIFO_ALMOST_EMPTY = S_RESET;
        tx_fifo_buffer = NULL;
    }

#ifndef RX_FIFO_THR_WA
    /* The IRQ_TX_FIFO_ALMOST_EMPTY notifies an nearly empty TX fifo */
    if(x_irq_status.IRQ_TX_FIFO_ALMOST_EMPTY) {
#ifdef DEBUG_IRQ
        uint32_t *tmp = (uint32_t*)&x_irq_status;
        debug_if(!((*tmp) & IRQ_TX_FIFO_ALMOST_EMPTY_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
        debug_if(!_s2lp_tx_started, "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
        debug_if(tx_fifo_buffer == NULL, "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif

        int8_t fifo_available = S2LP_MAX_FIFO_LEN/2; // fill-up half fifo
        int8_t to_send = (tx_fifo_remaining > fifo_available) ? fifo_available : tx_fifo_remaining;

        tx_fifo_remaining -= to_send;

        /* Fill FIFO Buffer */
        if(to_send > 0) {
            spi_write_linear_fifo(to_send, (uint8_t*)&tx_fifo_buffer[tx_buffer_pos]);
        }
        tx_buffer_pos += to_send;
    }
#endif // !RX_FIFO_THR_WA

    /* TX FIFO underflow/overflow error */
    if(x_irq_status.IRQ_TX_FIFO_ERROR) {
#ifdef DEBUG_IRQ
        uint32_t *tmp = (uint32_t*)&x_irq_status;
        debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
        debug_if(!((*tmp) & IRQ_TX_FIFO_ERROR_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
        if(_s2lp_tx_started) {
            _s2lp_tx_started = false;
            /* call user callback */
            if(_current_irq_callback) {
                _current_irq_callback(TX_ERR);
            }
        }

        /* reset data still to be sent */
        tx_fifo_remaining = 0;
    }

    /* The IRQ_RX_DATA_READY notifies a new packet arrived */
    if(x_irq_status.IRQ_RX_DATA_READY) {
#ifdef DEBUG_IRQ
        uint32_t *tmp = (uint32_t*)&x_irq_status;
        debug_if(!((*tmp) & IRQ_RX_DATA_READY_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif

        if(!_is_receiving) { // spurious irq?!? (betzw: see comments on macro 'RX_FIFO_THR_WA'!)
#ifdef HEAVY_DEBUG
            debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
#endif
        } else {
            _is_receiving = false; // Finished receiving
            stop_rx_timeout();

            s2lp_rx_len = pkt_basic_get_received_pkt_length();

#ifdef DEBUG_IRQ
            debug_if(!(s2lp_rx_len <= MAX_PACKET_LEN), "\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
#endif

            if(s2lp_rx_len <= MAX_PACKET_LEN) {
                uint8_t to_receive = s2lp_rx_len - _s2lp_rx_pos;
                if(to_receive > 0) {
                    spi_read_linear_fifo(to_receive, &s2lp_rx_buf[_s2lp_rx_pos]);
                    _s2lp_rx_pos += to_receive;
                }
            }

            cmd_strobe(S2LP_STROBE_FRX);

            last_rssi = qi_get_rssi(); //MGR
            last_sqi  = qi_get_sqi();  //MGR

            /* call user callback */
            if((_s2lp_rx_pos == s2lp_rx_len) && _current_irq_callback) {
                _current_irq_callback(RX_DONE);
            }

            /* Disable handling of other RX flags */
            x_irq_status.IRQ_RX_FIFO_ALMOST_FULL = S_RESET;
        }
    }

#ifndef RX_FIFO_THR_WA
    /* RX FIFO almost full */
    if(x_irq_status.IRQ_RX_FIFO_ALMOST_FULL) {
#ifdef DEBUG_IRQ
        uint32_t *tmp = (uint32_t*)&x_irq_status;
        debug_if(!((*tmp) & IRQ_RX_FIFO_ALMOST_FULL_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
        if(!_is_receiving) { // spurious irq?!?
#ifdef DEBUG_IRQ
            debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
#endif
        } else {
            uint8_t fifo_available = linear_fifo_read_num_elements_rx_fifo();
            if((fifo_available + _s2lp_rx_pos) <= MAX_PACKET_LEN) {
                spi_read_linear_fifo(fifo_available, &s2lp_rx_buf[_s2lp_rx_pos]);
                _s2lp_rx_pos += fifo_available;
            } else {
#ifdef DEBUG_IRQ
                debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
#endif
            }
        }
    }
#endif // !RX_FIFO_THR_WA

    /* Reception errors */
    if((x_irq_status.IRQ_RX_FIFO_ERROR) || (x_irq_status.IRQ_RX_DATA_DISC)) {
#ifdef DEBUG_IRQ
        uint32_t *tmp = (uint32_t*)&x_irq_status;
        debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
        debug_if(!((*tmp) & (IRQ_RX_FIFO_ERROR_MASK | IRQ_RX_DATA_DISC_MASK)), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
        rx_timeout_handler();
        if(_s2lp_tx_started) {
            _s2lp_tx_started = false;
            /* call user callback */
            if(_current_irq_callback) {
                _current_irq_callback(TX_ERR);
            }
        }
    }

    /* The IRQ_VALID_SYNC is used to notify a new packet is coming */
    if(x_irq_status.IRQ_VALID_SYNC) {
#ifdef DEBUG_IRQ
        uint32_t *tmp = (uint32_t*)&x_irq_status;
        debug_if(!((*tmp) & IRQ_VALID_SYNC_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
        /* betzw - NOTE: there is a race condition between S2LP receiving packets and
         *               the MCU trying to send a packet, which gets resolved in favor of
         *               sending.
         */
        if(_s2lp_tx_started) {
#ifdef DEBUG_IRQ
            debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
#endif
        } else {
            _is_receiving = true;
            start_rx_timeout();
        }
    }
}
