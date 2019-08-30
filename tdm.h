/// initialise tdm subsystem
///
void tdm_init(void);

// tdm main loop
///
void tdm_serial_loop(void);

/// report tdm timings
///
void tdm_report_timing(void);

/// dispatch a remote AT command
void tdm_remote_at(void);

/// change tdm phase (for testing recovery)
void tdm_change_phase(void);

/// show RSSI information
void tdm_show_rssi(void);

/// the long term duty cycle we are aiming for
int duty_cycle;

/// the LBT threshold
int lbt_rssi;