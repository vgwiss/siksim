import SiKRadio
class Radio:
    def __init__(self, transmit, receive, numChannels):
        self.transmit_channel = transmit
        self.receive_channel = receive
        self.num_fh_channels = numChannels
        self.have_radio_lock = False

	def fhop_window_change():
		self.transmit_channel = (self.transmit_channel + 1) % self.num_fh_channels
		if self.have_radio_lock:
			self.receive_channel = self.transmit_channel
		elif (self.transmit_channel == 0):
			#when we don't have lock, the receive channel only
 			#changes when the transmit channel wraps
			self.receive_channel = (self.receive_channel + 1) % self.num_fh_channels

    def fhop_set_locked(self, locked):
        self.have_radio_lock = locked
        if (self.have_radio_lock):
            self.transmit_channel = self.receive_channel
        else:
            self.receive_channel = (self.receive_channel+1) % self.num_fh_channels

def doTick(radio1, radio2):
    if radio1.receive_channel == radio2.transmit_channel:
        radio1.fhop_set_locked(True)
        radio2.fhop_set_locked(True)
    elif radio1.transmit_channel == radio2.receive_channel:
        radio1.fhop_set_locked(True)
        radio2.fhop_set_locked(True)
    else:
        radio1.fhop_set_locked(False)
        radio2.fhop_set_locked(False)
    print("GCS ", radio1.transmit_channel, radio1.receive_channel)
    print("Air ", radio2.transmit_channel, radio2.receive_channel)

# TX channel, RX channel, #of channels
gcs = Radio(10, 2, 20)
air = Radio(3, 6, 20)
doTick(gcs, air)
doTick(gcs, air)
doTick(gcs, air)
doTick(gcs, air)
doTick(gcs, air)
doTick(gcs, air)
doTick(gcs, air)
doTick(gcs, air)
doTick(gcs, air)
doTick(gcs, air)
doTick(gcs, air)
