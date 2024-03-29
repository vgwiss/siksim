// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  o Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  o Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

///
/// @file	freq_hopping.h
///
/// Prototypes for the frequency hopping manager
///

#ifndef _FREQ_HOPPING_H_
#define _FREQ_HOPPING_H_

#define MAX_FREQ_CHANNELS 50

/// Randomly shuffle fixed variables for entoropy
///
void shuffleRand(void);

void showChannels(void);

/// initialise frequency hopping logic
///
void fhop_init(void);

/// tell the TDM code what channel to transmit on
///
/// @return		The channel that we should be transmitting on.
///
char fhop_transmit_channel(void);

/// tell the TDM code what channel to receive on
///
/// @return		The channel that we should be receiving on.
//
char fhop_receive_channel(void);

/// called when the transmit window flips
///
void fhop_window_change(void);

/// called when we get or lose radio lock
///
/// @param locked	True if we have gained lock, false if we have lost it.
///
void fhop_set_locked(bool locked);

/// how many channels are we hopping over
char num_fh_channels;

#endif // _FREQ_HOPPING_H_
