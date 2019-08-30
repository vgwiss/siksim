// #include "radio.h"
// #include "tdm.h"
// #include "timer.h"
// #include "packet.h"
// #include "golay.h"
#include "freq_hopping.h"
// #include "crc.h"
//#include "serial.h"
#include "tdm.h"

#define TDM_TRANSMIT 0
#define TDM_SILENCE1 1
#define TDM_RECEIVE  2
#define TDM_SILENCE2 3
const unsigned char MAX_PACKET_LENGTH = 252;

int tdmState;
unsigned char pbuf[MAX_PACKET_LENGTH];

/// how many 16usec ticks are remaining in the current state
int tdm_state_remaining;

/// This is enough to hold at least 3 packets and is based
/// on the configured air data rate.
int tx_window_width;

/// the maximum data packet size we can fit
int max_data_packet_length;

/// the silence period between transmit windows
/// This is calculated as the number of ticks it would take to transmit
/// two zero length packets
int silence_period;

/// whether we can transmit in the other radios transmit window
/// due to the other radio yielding to us
char bonus_transmit;

/// whether we have yielded our window to the other radio
char transmit_yield;

// activity indication
// when the 16 bit timer2_tick() value wraps we check if we have received a
// packet since the last wrap (ie. every second)
// If we have the green radio LED is held on.
// Otherwise it blinks every 1 seconds. The received_packet flag
// is set for any received packet, whether it contains user data or
// not.
char blink_state;
char received_packet;

/// the latency in 16usec timer2 ticks for sending a zero length packet
int packet_latency;

/// the time in 16usec ticks for sending one byte
int ticks_per_byte;

/// number of 16usec ticks to wait for a preamble to turn into a packet
/// This is set when we get a preamble interrupt, and causes us to delay
/// sending for a maximum packet latency. This is used to make it more likely
/// that two radios that happen to be exactly in sync in their sends
/// will eventually get a packet through and get their transmit windows
/// sorted out
int transmit_wait;

/// the long term duty cycle we are aiming for
int duty_cycle;

/// the average duty cycle we have been transmitting
double average_duty_cycle;

/// duty cycle offset due to temperature
char duty_cycle_offset;

/// set to true if we need to wait for our duty cycle average to drop
char duty_cycle_wait;

/// how many ticks we have transmitted for in this TDM round
int transmitted_ticks;

/// the LDB (listen before talk) RSSI threshold
char lbt_rssi;

/// how long we have listened for for LBT
int lbt_listen_time;

/// how long we have to listen for before LBT is OK
int lbt_min_time;

/// random addition to LBT listen time (see European regs)
int lbt_rand;

/// test data to display in the main loop. Updated when the tick
/// counter wraps, zeroed when display has happened
char test_display;

/// set when we should send a statistics packet on the next round
char send_statistics;

/// set when we should send a MAVLink report pkt
extern bool seen_mavlink;

struct tdm_trailer {
	int window:13;
	int command:1;
	int bonus:1;
	int resend:1;
};

struct tdm_trailer trailer;

/// buffer to hold a remote AT command before sending
char send_at_command;
//char remote_at_cmd[AT_CMD_MAXLEN + 1];

// initialise the TDM subsystem
void
tdm_init(void)
{
  int i;
  char air_rate = radio_air_rate();
  long window_width;
  
#define REGULATORY_MAX_WINDOW (((1000000UL/16)*4)/10)
#define LBT_MIN_TIME_USEC 5000

	// tdm_build_timing_table();

	// calculate how many 16usec ticks it takes to send each byte
	ticks_per_byte = (8+(8000000UL/(air_rate*1000UL)))/16;
        ticks_per_byte++;

	// calculate the minimum packet latency in 16 usec units
	// we initially assume a preamble length of 40 bits, then
	// adjust later based on actual preamble length. This is done
	// so that if one radio has antenna diversity and the other
	// doesn't, then they will both using the same TDM round timings
	packet_latency = (8+(10/2)) * ticks_per_byte + 13;

	if (feature_golay) {
		max_data_packet_length = (MAX_PACKET_LENGTH/2) - (6+sizeof(trailer));

		// golay encoding doubles the cost per byte
		ticks_per_byte *= 2;

		// and adds 4 bytes
		packet_latency += 4*ticks_per_byte;
	} else {
		max_data_packet_length = MAX_PACKET_LENGTH - sizeof(trailer);
	}

	// set the silence period to two times the packet latency
        silence_period = 2*packet_latency;

        // set the transmit window to allow for 3 full sized packets
	window_width = 3*(packet_latency+(max_data_packet_length*(uint32_t)ticks_per_byte));

        // min listen time is 5ms
        lbt_min_time = LBT_MIN_TIME_USEC/16;
        
	// if LBT is enabled, we need at least 3*5ms of window width
	if (lbt_rssi != 0) {
		window_width = constrain(window_width, 3*lbt_min_time, window_width);
	}

	// the window width cannot be more than 0.4 seconds to meet US
	// regulations
	if (window_width >= REGULATORY_MAX_WINDOW && num_fh_channels > 1) {
		window_width = REGULATORY_MAX_WINDOW;
	}

	// user specified window is in milliseconds
	if (window_width > param_get(PARAM_MAX_WINDOW)*(1000/16)) {
		window_width = param_get(PARAM_MAX_WINDOW)*(1000/16);
	}

	// make sure it fits in the 13 bits of the trailer window
	if (window_width > 0x1fff) {
		window_width = 0x1fff;
	}

	tx_window_width = window_width;

	// now adjust the packet_latency for the actual preamble
	// length, so we get the right flight time estimates, while
	// not changing the round timings
	packet_latency += ((settings.preamble_length-10)/2) * ticks_per_byte;

	// tell the packet subsystem our max packet size, which it
	// needs to know for MAVLink packet boundary detection
	i = (tx_window_width - packet_latency) / ticks_per_byte;
	if (i > max_data_packet_length) {
		i = max_data_packet_length;
	}
	packet_set_max_xmit(i);
}

void tdm_show_rssi(void)
{
	/**printf("L/R RSSI: %u/%u  L/R noise: %u/%u pkts: %u ",
	       (unsigned)statistics.average_rssi,
	       (unsigned)remote_statistics.average_rssi,
	       (unsigned)statistics.average_noise,
	       (unsigned)remote_statistics.average_noise,
	       (unsigned)statistics.receive_count);
    printf(" txe=%u rxe=%u stx=%u srx=%u ecc=%u/%u temp=%d dco=%u\n",
	       (unsigned)errors.tx_errors,
	       (unsigned)errors.rx_errors,
	       (unsigned)errors.serial_tx_overflow,
	       (unsigned)errors.serial_rx_overflow,
	       (unsigned)errors.corrected_errors,
	       (unsigned)errors.corrected_packets,
	       (int)radio_temperature(),
	       (unsigned)duty_cycle_offset);
	statistics.receive_count = 0; **/
}

void display_test_output(void)
{
  if (test_display & AT_TEST_RSSI) {
    tdm_show_rssi();
  }
}


/// estimate the flight time for a packet given the payload size
///
/// @param packet_len		payload length in bytes
///
/// @return			flight time in 16usec ticks
static int flight_time_estimate(int packet_len)
{
  return packet_latency + (packet_len * ticks_per_byte);
}


/// synchronise tx windows
///
/// we receive a 16 bit value with each packet which indicates how many
/// more 16usec ticks the sender has in their transmit window. The
/// sender has already adjusted the value for the flight time
///
/// The job of this function is to adjust our own transmit window to
/// match the other radio and thus bring the two radios into sync
///
static void
sync_tx_windows(unsigned char packet_length)
{
  int old_state = tdmState;
  int old_remaining = tdm_state_remaining;
  
  if (trailer.bonus) {
    // the other radio is using our transmit window
    // via yielded ticks
    if (old_state == TDM_SILENCE1) {
      // This can be caused by a packet
      // taking longer than expected to arrive.
      // don't change back to transmit state or we
      // will cause an extra frequency change which
      // will get us out of sequence
      tdm_state_remaining = silence_period;
    } else if (old_state == TDM_RECEIVE || old_state == TDM_SILENCE2) {
      // this is quite strange. We received a packet
      // so we must have been on the right
      // frequency. Best bet is to set us at the end
      // of their silence period
      tdmState = TDM_SILENCE2;
      tdm_state_remaining = 1;
    } else {
      tdmState = TDM_TRANSMIT;
      tdm_state_remaining = trailer.window;
    }
  } else {
    // we are in the other radios transmit window, our
    // receive window
    tdmState = TDM_RECEIVE;
    tdm_state_remaining = trailer.window;
  }
  
  // if the other end has sent a zero length packet and we are
  // in their transmit window then they are yielding some ticks to us.
  bonus_transmit = (tdmState == TDM_RECEIVE && packet_length==0);
  
  // if we are not in transmit state then we can't be yielded
  if (tdmState != TDM_TRANSMIT) {
    transmit_yield = 0;
  }
  
  if (at_testmode & AT_TEST_TDM) {
    int delta;
    delta = old_remaining - tdm_state_remaining;
    if (old_state != tdmState ||
        delta > (int)packet_latency/2 ||
        delta < -(int)packet_latency/2) {
      printf("TDM: %u/%u len=%u ",
          (unsigned)old_state,
          (unsigned)tdmState,
          (unsigned)packet_length);
      printf(" delta: %d\n",(int)delta);
    }
  }
}

/// update the TDM state machine
///
static void
tdm_state_update(int tdelta)
{
  // update the amount of time we are waiting for a preamble
  // to turn into a real packet
  if (tdelta > transmit_wait) {
    transmit_wait = 0;
  } else {
    transmit_wait -= tdelta;
  }
  
  // have we passed the next transition point?
  while (tdelta >= tdm_state_remaining) {
    // advance the tdm state machine
    tdmState = (tdmState+1) % 4;
    
    // work out the time remaining in this state
    tdelta -= tdm_state_remaining;
    
    if (tdmState == TDM_TRANSMIT || tdmState == TDM_RECEIVE) {
      tdm_state_remaining = tx_window_width;
    } else {
      tdm_state_remaining = silence_period;
    }
    
    // change frequency at the start and end of our transmit window
    // this maximises the chance we will be on the right frequency
    // to match the other radio
    if (tdmState == TDM_TRANSMIT || tdmState == TDM_SILENCE1) {
      fhop_window_change();
      radio_receiver_on();
      
      if (num_fh_channels > 1) {
        // reset the LBT listen time
        lbt_listen_time = 0;
        lbt_rand = 0;
      }
    }
    
    if (tdmState == TDM_TRANSMIT && (duty_cycle - duty_cycle_offset) != 100) {
      // update duty cycle averages
      average_duty_cycle = (0.95*average_duty_cycle) + (0.05*(100.0*transmitted_ticks)/(2*(silence_period+tx_window_width)));
      transmitted_ticks = 0;
      duty_cycle_wait = (average_duty_cycle >= (duty_cycle - duty_cycle_offset));
    }
    
    // we lose the bonus on all state changes
    bonus_transmit = 0;
    
    // reset yield flag on all state changes
    transmit_yield = 0;
    
    // no longer waiting for a packet
    transmit_wait = 0;
  }
  
  tdm_state_remaining -= tdelta;
}

/// change tdm phase
///
void
tdm_change_phase(void)
{
  tdmState = (tdmState+2) % 4;
}

/// blink the radio LED if we have not received any packets
///
static void
link_update(void)
{
  char unlock_count = 10, temperature_count;
  if (received_packet) {
    unlock_count = 0;
    received_packet = false;
  } else {
    unlock_count++;
  }
  
  if (unlock_count < 2) {
  } else {
    blink_state = !blink_state;
  }
  
  if (unlock_count > 40) {
    // if we have been unlocked for 20 seconds
    // then start frequency scanning again
    
    unlock_count = 5;
    // randomise the next transmit window using some
    // entropy from the radio if we have waited
    // for a full set of hops with this time base
    if (timer_entropy() & 1) {
      int old_remaining = tdm_state_remaining;
      if (tdm_state_remaining > silence_period) {
        tdm_state_remaining -= packet_latency;
      } else {
        tdm_state_remaining = 1;
      }
      if (at_testmode & AT_TEST_TDM) {
        printf("TDM: change timing %u/%u\n",
                (unsigned)old_remaining,
                (unsigned)tdm_state_remaining);
      }
    }
    
    if (at_testmode & AT_TEST_TDM) {
      printf("TDM: scanning\n");
    }
    fhop_set_locked(false);
  }
  
  if (unlock_count != 0) {
    statistics.average_rssi = (radio_last_rssi() + 3*(int)statistics.average_rssi)/4;
    
    // reset statistics when unlocked
    statistics.receive_count = 0;
  }
  
  if (unlock_count > 5) {
    memset(&remote_statistics, 0, sizeof(remote_statistics));
  }
  
  test_display = at_testmode;
  send_statistics = 1;
  //printf("ST");
  }
}

// dispatch an AT command to the remote system
void
tdm_remote_at(void)
{
  memcpy(remote_at_cmd, at_cmd, strlen(at_cmd)+1);
  send_at_command = true;
}

// handle an incoming at command from the remote radio
// 
// Return true if returning a pbuf that needs to be sent to output
//        false if data is going out to the other modem
static bool 
handle_at_command(__pdata uint8_t len)
{
  if (len < 2 || len > AT_CMD_MAXLEN ||
      pbuf[0] != (uint8_t)'R' ||
      pbuf[1] != (uint8_t)'T') {
    return true;
  }
  
  // setup the command in the at_cmd buffer
  memcpy(at_cmd, pbuf, len);
  at_cmd[len] = 0;
  at_cmd[0] = 'A'; // replace 'R'
  at_cmd_len = len;
  at_cmd_ready = true;
  
  // run the AT command, capturing any output to the packet
  // buffer
  // this reply buffer will be sent at the next opportunity
  printf_start_capture(pbuf, sizeof(pbuf));
  at_command();
  len = printf_end_capture();
  if (len > 0) {
    packet_inject(pbuf, len);
  }
 return false;
}

/// main loop for time division multiplexing transparent serial
///
void
tdm_serial_loop(void)
{
  int	len;
  int   tnow, tdelta;
  int   max_xmit;
  int   last_t = timer2_tick();
  int   last_link_update = last_t;
  
  for (;;) {
    // give the AT command processor a chance to handle a command
    at_command();
    
    // display test data if needed
    if (test_display) {
      display_test_output();
      test_display = 0;
    }
    
    if (seen_mavlink && feature_mavlink_framing && !at_mode_active) {
      seen_mavlink = false;
      MAVLink_report();
    }
    
    // set right receive channel
    radio_set_channel(fhop_receive_channel());
    
    // get the time before we check for a packet coming in
    tnow = timer2_tick();
    
    // see if we have received a packet
    if (radio_receive_packet(&len, pbuf)) {
      
      // int k = 0;
      // printf("\r\n");
      // for(k; k < len; k++)
      // {
      //   printf("%x",pbuf[k]);
      // }

      // update the activity indication
      received_packet = true;
      fhop_set_locked(true);
      
      // update filtered RSSI value and packet stats
      statistics.average_rssi = (radio_last_rssi() + 7*(uint16_t)statistics.average_rssi)/8;
      statistics.receive_count++;
      
      // we're not waiting for a preamble
      // any more
      transmit_wait = 0;
      
      if (len < 2) {
        // not a valid packet. We always send
        // two control bytes at the end of every packet
        continue;
      }
      
      //printf("Received Packet\r\n");
      
      // extract control bytes from end of packet
      memcpy(&trailer, &pbuf[len-sizeof(trailer)], sizeof(trailer));
      len -= sizeof(trailer);
      
      if(trailer.window == 0 && len != 0) {
        // its a control packet
        if (len == sizeof(struct statistics)) {
          memcpy(&remote_statistics, pbuf, len);
        }
        
        // don't count control packets in the stats
        statistics.receive_count--;
      } else if (trailer.window != 0) {
        // sync our transmit windows based on
        // received header
        sync_tx_windows(len);
        last_t = tnow;
        

	// Send data to console (serial buffers) if following conditions met
	// If is a command and data is destined to THIS modem
	// OR
	// data is present, not a command, not a dup and not in AT Mode
	//
	// Question: Are we happy to blink Activity lights for RT command results?
        if ((trailer.command == 1 && handle_at_command(len)) 
            ||
            (len != 0 && trailer.command == 0 &&
             !packet_is_duplicate(len, pbuf, trailer.resend) &&
             !at_mode_active
            )) 
        {
             // its user data - send it out
             // the serial port
             serial_write_buf(pbuf, len);
        }
      }
      continue;
    }
    
    // see how many 16usec ticks have passed and update
    // the tdm state machine. We re-fetch tnow as a bad
    // packet could have cost us a lot of time.
    tnow = timer2_tick();
    tdelta = tnow - last_t;
    tdm_state_update(tdelta);
    last_t = tnow;
    
    // update link status every 0.5s
    if(tnow - last_link_update > 32768) {
      link_update();
      last_link_update = tnow;
    }
    

    if (lbt_rssi != 0) {
      // implement listen before talk
      if (radio_current_rssi() < lbt_rssi) {
        lbt_listen_time += tdelta;
      } else {
        lbt_listen_time = 0;
        if (lbt_rand == 0) {
          lbt_rand = ((uint16_t)rand()) % lbt_min_time;
        }
      }
      if (lbt_listen_time < lbt_min_time + lbt_rand) {
        // we need to listen some more
        printf("Need to listen more\r\n");
        continue;
      }
    }
    
    // we are allowed to transmit in our transmit window
    // or in the other radios transmit window if we have
    // bonus ticks
    if (tdmState != TDM_TRANSMIT) {
      continue;
    }		
    
    if (transmit_yield != 0) {
      // we've give up our window
      continue;
    }
    
    if (transmit_wait != 0) {
      // we're waiting for a preamble to turn into a packet
      continue;
    }
    
    if (!received_packet &&
          radio_preamble_detected() ||
          radio_receive_in_progress()) {
      // a preamble has been detected. Don't
      // transmit for a while
      transmit_wait = packet_latency;
      continue;
    }
    
    // sample the background noise when it is out turn to
    // transmit, but we are not transmitting,
    // averaged over around 4 samples
    statistics.average_noise = (radio_current_rssi() + 3*(uint16_t)statistics.average_noise)/4;
    
    if(duty_cycle_wait) {
      // we're waiting for our duty cycle to drop
      continue;
    }
    
    // how many bytes could we transmit in the time we
    // have left?
    if (tdm_state_remaining < packet_latency) {
      // none ....
      continue;
    }
    max_xmit = (tdm_state_remaining - packet_latency) / ticks_per_byte;
    //printf("%u\r\n", max_xmit);
    if(max_xmit < PACKET_OVERHEAD) {
      // can't fit the trailer in with a byte to spare
      continue;
    }
    //max_xmit -= PACKET_OVERHEAD;
    max_xmit -= sizeof(trailer)+1;
    
    if (max_xmit > max_data_packet_length) {
      max_xmit = max_data_packet_length;
    }
    
    // ask the packet system for the next packet to send
    if (send_at_command && 
            max_xmit >= strlen(remote_at_cmd)) {
      // send a remote AT command
      len = strlen(remote_at_cmd);
      memcpy(pbuf, remote_at_cmd, len);
      trailer.command = 1;
      send_at_command = false;
    } else {
      // get a packet from the serial port
      len = packet_get_next(max_xmit, pbuf);
      //printf("Len = %x", len);

      if (len > 0) {
         trailer.command = packet_is_injected();
      } else {
         trailer.command = 0;
      }
    }
    
    if (len > max_data_packet_length) {
      //panic("oversized tdm packet");
    }
    
    trailer.bonus = (tdmState == TDM_RECEIVE);
    trailer.resend = packet_is_resend();
    
    if (tdmState == TDM_TRANSMIT &&
            len == 0 &&
            send_statistics &&
            max_xmit >= sizeof(statistics)) {
      // send a statistics packet
      send_statistics = 0;
      memcpy(pbuf, &statistics, sizeof(statistics));
      len = sizeof(statistics);
      
      // mark a stats packet with a zero window
      trailer.window = 0;
      trailer.resend = 0;
    } else {
      // calculate the control word as the number of
      // 16usec ticks that will be left in this
      // tdm state after this packet is transmitted
      
      trailer.window = (int)(tdm_state_remaining - flight_time_estimate(len+sizeof(trailer)));
    }
    
    // set right transmit channel
    radio_set_channel(fhop_transmit_channel());
    
    memcpy(&pbuf[len], &trailer, sizeof(trailer));
    
    if(len != 0 && trailer.window != 0) {
      // show the user that we're sending real data
      //LED_ACTIVITY = LED_ON;
    }
    
    if (len == 0) {
      // sending a zero byte packet gives up
      // our window, but doesn't change the
      // start of the next window
      transmit_yield = 1;
     // printf("y");
    }

    else{
      //printf("n\r\n");
    }
    //printf("%x", transmit_yield);
    // after sending a packet leave a bit of time before
    // sending the next one. The receivers don't cope well
    // with back to back packets
    transmit_wait = packet_latency;
    
    // if we're implementing a duty cycle, add the
    // transmit time to the number of ticks we've been transmitting
    if((duty_cycle - duty_cycle_offset) != 100) {
      transmitted_ticks += flight_time_estimate(len+sizeof(trailer));
    }
    
    // start transmitting the packet
    if (!radio_transmit(len + sizeof(trailer), pbuf, tdm_state_remaining + (silence_period/2)) &&
        len != 0 && trailer.window != 0 && trailer.command == 0) {
      packet_force_resend();
    }
    
    if (lbt_rssi != 0) {
      // reset the LBT listen time
      lbt_listen_time = 0;
      lbt_rand = 0;
    }
    
    if (len != 0 && trailer.window != 0) {
      //LED_ACTIVITY = LED_OFF;
    }

    // set right receive channel
    radio_set_channel(fhop_receive_channel());
    
    // re-enable the receiver
    radio_receiver_on();
    
  }
}

int main () {}
