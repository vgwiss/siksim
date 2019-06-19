class Radio:
	transmit_channel = 0
	receive_channel = 0
	num_fh_channels = 20
	have_radio_lock = True
	def __init__(self, transmit, receive, numChannels):
		transmit_channel = transmit
		receive_channel = receive
		num_fh_channels = num_fh_channels

	def fhop_window_change():
		global transmit_channel
		global num_fh_channels
		global receive_channel
		global have_radio_lock

		transmit_channel = (transmit_channel + 1) % num_fh_channels
		if have_radio_lock:
			receive_channel = transmit_channel
		elif (transmit_channel == 0):
			#when we don't have lock, the receive channel only
 			#changes when the transmit channel wraps
			receive_channel = (receive_channel + 1) % num_fh_channels

		def fhop_set_locked(locked):
			global have_radio_lock
			global transmit_channel
			global receive_channel
			global num_fh_channels

			have_radio_lock = locked;
			if (have_radio_lock):
				# we have just received a packet, so we know the
				# other radios transmit channel must be our receive
				# channel
				transmit_channel = receive_channel;
			else:
				# try the next receive channel
				receive_channel = (receive_channel+1) % num_fh_channels;
