#
/*
 *    Copyright (C) 2020, 2022
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Computing
 *
 *    This file is part of the SDRuno fax plugin
 *
 *    fax plugin is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation as version 2 of the License.
 *
 *    fax plugin is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with fax plugin; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include	<sstream>
#include	<unoevent.h>
#include	<iunoplugincontroller.h>
#include	<vector>
#include	<sstream>
#include	<complex>
#include	<chrono>
#include        <Windows.h>
#include	<stdio.h>
#include	<stdlib.h>

#define  _USE_MATH_DEFINES
#include        <math.h>

#include "SDRunoPlugin_fax.h"
#include "SDRunoPlugin_faxUi.h"
//
//	fax specfics

#include	"fax-bandfilter.h"
#include	"fax-shifter.h"
//#include	"utilities.h"
#include        "up-filter.h"
//
////	for the dump-filename
#include	<time.h>
#include	<ctime>
#define	FAX_IF 0

faxParams _faxParams [] = {
	{"Wefax288", 288, 675, 450, false, 120, 600},
	{"Wefax576", 576, 300, 450, false, 120, 1200},
	{"HamColor", 204, 200, 450, true,  360, 0},	// not visible
	{"Ham288b",  240, 675, 450, false, 240, 0},	// not visible
	{"Color240", 288, 200, 450, true,  240, 0},	// not visible
	{"FAX480",   204, 500, 450, false, 480, 0},	// not visible
	{nullptr,     -1,  -1,  -1, false,  -1, 0}	
};

static inline
float Minimum (float x, float y) {
	return x < y ? x : y;
}

static inline
std::complex<float> cmul (std::complex<float> x, float y) {
	return std::complex<float> (real (x) * y, imag (x) * y);
}

static inline
std::complex<float> cdiv (std::complex<float> x, float y) {
	return std::complex<float> (real (x) / y, imag (x) / y);
}

static inline
bool	isWhite(int16_t x) {
	return x >= 128;
}

static inline
float	clamp (float X, float Min, float Max) {
	if (X > Max)
	   return Max;
	if (X < Min)
	   return Min;
	return X;
}

	SDRunoPlugin_fax::
	              SDRunoPlugin_fax (IUnoPluginController& controller):
	                                   IUnoPlugin	(controller),
	                                   m_form	(*this, controller),
	                                   m_worker	(nullptr),
	                                   inputBuffer	(32 * 32768),
	                                   passbandFilter (11,
	                                                   -2000,
	                                                   +2000,
	                                                   INRATE),
	                                   theDecimator (INRATE / WORKING_RATE),
	                                   audioFilter  (11, 1500, INRATE),
	                                   audioBuffer (8 * 32768),
	                                   localMixer (WORKING_RATE),
	                                   faxLineBuffer (600) {
	m_controller            = &controller;
	running. store (false);

	overflow. store (0);	// could be just a constant
	faxAudioRate	= 192000;	// in this configuration
	carrier		= FAX_IF;
	resetFlag. store (false);
	cheatFlag. store (false);
	deviation	= 400;		// default
//
	correcting.store  (false);
	setCorrection		= false;
	saveContinuous		= false;
	saveSingle		= false;
	running. store (false);
	m_controller    -> RegisterAudioProcessor (0, this);
	m_controller	-> SetDemodulatorType (0,
	                           IUnoPluginController::DemodulatorIQOUT);
//
//	and off we go
	m_worker = new std::thread (&SDRunoPlugin_fax::WorkerFunction, this);
}

	SDRunoPlugin_fax::~SDRunoPlugin_fax () {	
	running.store(false);
	m_worker 	-> join();
	m_controller    -> UnregisterAudioProcessor (0, this);
	delete	        m_worker;
}

void	SDRunoPlugin_fax::
	         StreamProcessorProcess (channel_t channel,
	                                 Complex* buffer,
	                                 int	length,
	                                 bool& modified) {
	(void)channel; (void)buffer; (void)length;
	modified = false;
}

void	SDRunoPlugin_fax::AudioProcessorProcess (channel_t channel,
	                                         float* buffer,
	                                         int length,
	                                         bool& modified) {
//	Handling IQ input, note that SDRuno interchanges I and Q elements
	if (!modified) {
	   for (int i = 0; i < length; i++) {
	      std::complex<float> sample =
	                    std::complex<float> (buffer [2 * i + 1],
	                                         buffer [2 * i]);
		 sample = passbandFilter.Pass(sample);
	     inputBuffer. putDataIntoBuffer (&sample, 1);
	   }
	}
	if (audioBuffer. GetRingBufferReadAvailable () >= length) {
	   std::complex<float> audioSample;
	   for (int i = 0; i < length; i ++) {
	      audioBuffer. getDataFromBuffer (&audioSample, 1);
	      buffer [2 * i + 1] = real (audioSample);
	      buffer [2 * i    ] = imag (audioSample);
	   }
	   modified = true;
	}
}

void	SDRunoPlugin_fax::HandleEvent (const UnoEvent& ev) {
	switch (ev. GetType ()) {
	   case UnoEvent::FrequencyChanged:
	      break;

	   case UnoEvent::CenterFrequencyChanged:
	      break;

	   default:
	      m_form. HandleEvent (ev);
	      break;
	}
}
//
//	The basic idea is to handle (serious) changes to the
//	state, i.e. changing the deviation or changing the IOC,
//	here, just testing on the start of the loop
//	So, by transforming asycnhronous events to synmchronous
//	events we simplify the code dramatically
std::complex<float> buffer [WORKING_RATE / 10];
void	SDRunoPlugin_fax::WorkerFunction () {
int	currentDeviation	= 0;
std::vector<std::complex<float>> faxToneBuffer (faxAudioRate);
LowPassFIR	faxLowPass (FILTER_DEFAULT, deviation, WORKING_RATE);
int faxTonePhase;
int teller	= 0;
	Sleep (100);	// Just give others a chance
//
//	we mix the incoming - filtered - signal with a tone of 801 Hz
        for (int i = 0; i < faxAudioRate; i++) {
	       float phase = (float)i / faxAudioRate * 2 * M_PI;
	       faxToneBuffer[i] = std::complex<float> (cos (phase), sin (phase));
        } 
        faxTonePhase            = 0;

//	faxLowPass depends on the selection, it might change
	fax_setDeviation (m_form. getDeviation ());
	faxLowPass. newKernel (deviation);
	selected_IOC            = m_form. get_ioc       ();
	fax_setup       (selected_IOC);
	fax_setMode     (m_form. get_demodMode  ());
	fax_setColor    (m_form. get_faxColor   ());
	fax_setPhase    (m_form. get_phase      ());
//
//	All set, off we go
	running. store (true);
	while (running. load ()) {
	   while (running. load () &&
	              (inputBuffer. GetRingBufferReadAvailable () < INRATE / 10))
	      Sleep (1);
//
//	inputBuffer: samplerate inputRate
	   while (inputBuffer. GetRingBufferReadAvailable () > 1) {
	      std::complex<float> sample;
	      inputBuffer. getDataFromBuffer (&sample, 1);
		  std::complex<float> theTone =
			  cmul(sample * faxToneBuffer[faxTonePhase], 10.0f);
		  faxTonePhase = (faxTonePhase + 801) % faxAudioRate;
	      theTone = audioFilter. Pass (theTone);
	      audioBuffer. putDataIntoBuffer (&theTone, 1);
	      
		  if (theDecimator.Pass(sample, &sample)) {
			  buffer[teller] = sample;
			  teller++;

			  if (teller >= WORKING_RATE / 10)
				  break;
		  }
	   }

	   if (teller < WORKING_RATE / 10)
	      continue;

	   teller = 0;
	   if (!running. load ())
	      break;
//
//	handle flags, if set
	   if (resetFlag. load ()) {
	      fax_setup (selected_IOC);
	      carrier			= FAX_IF;
	      correcting. store (false);
	      setCorrection		= false;
	      saveContinuous		= false;
	      saveSingle		= false;
	      show_savingLabel	("Save Cont");
	      rawData. resize		(1024 * 1024);
	      resetFlag. store (false);
	      rawData. resize (1024 * 1024);
	   }
	   else
	   if (cheatFlag. load ()) {
	      if (theFax. faxState == SYNCED) {
	         theFax. faxState = FAX_DONE;
	         cheatFlag. store (false);
	         continue;
	      }
	      else {
	         theFax. bufferP 		= 0;
	         theFax. checkP 		= 0;
	         theFax. linesRecognized 	= 0;
	         theFax. alarmCount		= 0;
	         theFax. currentSampleIndex	= 0;
	         theFax. lastRow 		= 0;
	         theFax. stoppers 		= 0;
	         theFax. sampleOffset		= 0;
	         theFax. flipper		= 10;
	         theFax. faxState 		= SYNCED;
	         cheatFlag. store (false);
//	         clearScreen		();
	         show_faxState	("ON SYNC");
	      }
	   }
	
	   if (currentDeviation != deviation) {
	      faxLowPass. newKernel (deviation);
	      currentDeviation = deviation;
	   }

//
//	all (serious) asycnhronous changes are handled now,
//	so we process the incoming buffer
	   for (int i = 0; i < WORKING_RATE / 10; i++) {
	      int sampleValue;
	      std::complex<float> z =  buffer [i];
	      z			= localMixer. do_shift (z, carrier);
	      z			= faxLowPass. Pass (z);
	      sampleValue	= demodulate (z);
	      if (faxMode. phaseInvers)
	         sampleValue = 256 - sampleValue;
	      if (faxMode. faxColor == FAX_BLACKWHITE)
	         sampleValue = isWhite (sampleValue) ? 255 : 0;
	      processSample (sampleValue);
	   }  
	}

	show_faxState ("going down");
	Sleep (1000);
}
//
//	first, set up the values for the IOC selected. The
//	"run time" parameters are filled in on "APTSTART"
void	SDRunoPlugin_fax::fax_setup	(const std::string &s) {
faxParams	*myfaxParameters	= getFaxParams (s);

	faxMode. name		= s;
	faxMode. fax_IOC	= myfaxParameters -> IOC;
	faxMode. nrColumns	= M_PI * faxMode. fax_IOC;
	faxMode. nrLines	= myfaxParameters -> nrLines;
	faxMode. aptStartFreq	= myfaxParameters -> aptStart;
	faxMode. aptStopFreq	= myfaxParameters -> aptStop;
	faxMode. lpm		= myfaxParameters -> lpm;
	faxMode. phaseInvers	= false;
	faxMode. samplesperLine	= WORKING_RATE * 60 / faxMode. lpm;
	faxMode. faxColor	= myfaxParameters	->  color ?
	                                    FAX_COLOR: FAX_BLACKWHITE;
	faxMode. demodMode	=  FAX_FM;

	theFax. faxState	= APTSTART;
	theFax.  lastRow	= 0;	// will change	
	pixelStore. resize (faxMode. nrColumns * faxMode. nrLines);
	rawData. resize (1024 * 1024);
	faxLineBuffer. resize (5 * faxMode. samplesperLine);
	checkBuffer. resize (faxMode. samplesperLine  + 10);
}

static inline
bool	realWhite (int16_t x) {
	return x > 229;
}

static inline
bool	realBlack(int16_t x) {
	return x < 25;
}
//
void	SDRunoPlugin_fax::processSample (int sampleValue) {
int	baseP;
	
	switch (theFax. faxState) {
	   case	APTSTART:		// initialize the "theFax" record
	      show_faxState	("APTSTART");
	      theFax. bufferP		= 0;
	      theFax. checkP		= 0;
	      theFax. linesRecognized	= 0;
	      theFax. currentSampleIndex	= 0;
	      theFax. alarmCount	= 0;
	      theFax. lastRow		= 0;
	      theFax. stoppers		= 0;
	      theFax. sampleOffset	= 0;	
	      theFax. flipper		= 10;
	      clearScreen	();

	      faxLineBuffer [theFax. bufferP] = sampleValue;
	      theFax. bufferP ++;
	      theFax. faxState		= WAITING_FOR_START;
	      break;

	   case WAITING_FOR_START:
	      faxLineBuffer [theFax. bufferP] = sampleValue;
	      theFax. bufferP ++;
	      if (theFax. bufferP >= faxMode. samplesperLine) {
	         int upCrossings = checkFrequency (faxLineBuffer,
	                                           faxMode. samplesperLine,	
	                                           faxMode. aptStartFreq,
	                                           true);
	         if (upCrossings > 0) {
	            theFax. faxState = START_RECOGNIZED;
	            toRead = 6 * faxMode. samplesperLine;
	            theFax. bufferP = 0;
	         }
	         else {
	            for (int i = faxMode. samplesperLine / 10;
	                      i < faxMode. samplesperLine; i ++)
	               faxLineBuffer [i - faxMode. samplesperLine / 10] =
	                                               faxLineBuffer [i];
	            theFax. bufferP -= faxMode. samplesperLine / 10;
	         }
	      }
	      break;

	   case START_RECOGNIZED:
	      toRead --;
	      if (toRead <= 0) {
	         theFax. faxState	= WAITING_FOR_PHASE;
	         show_faxState ("PHASING");
	         theFax. alarmCount	= 0;
	         theFax. bufferP 	= 0;
	      }
	      break;

	   case WAITING_FOR_PHASE:
	      faxLineBuffer [theFax. bufferP] = sampleValue;
	      theFax. bufferP ++;
	      if (theFax. bufferP == faxLineBuffer. size () - 2) {
	         theFax. faxState = READ_PHASE;
	      }
	      break;

	   case READ_PHASE:
	      faxLineBuffer [theFax. bufferP] = sampleValue;
	      baseP = checkPhase (faxLineBuffer, 0, 0.90);
	      if (baseP >= 0) {
	         for (int i = baseP; i < faxMode. samplesperLine; i ++)
	            faxLineBuffer [i - baseP] = faxLineBuffer [i];
	         theFax. bufferP        = faxMode. samplesperLine - baseP;
	         theFax. currentSampleIndex = 0;
	         theFax. checkP		= 0;
	         theFax. faxState	= SYNCED;
	         show_faxState ("ON SYNC");
	         theFax. stoppers	= 0;
	         theFax. linesRecognized = 0;
	      }
	      else {
	         for (int i = faxMode. samplesperLine;
	                         i < faxLineBuffer. size (); i ++)
	            faxLineBuffer [i - faxMode. samplesperLine] =
	                                             faxLineBuffer [i];
	         theFax. bufferP = faxLineBuffer. size () -
	                                    faxMode. samplesperLine - 1;
	         theFax. alarmCount ++;
	         show_lineno (theFax. alarmCount);
	         if (theFax. alarmCount >= 15)
	            theFax. faxState = APTSTART;
	         else
	            theFax. faxState = WAITING_FOR_PHASE;
	      }
	      break;

	   case SYNCED:
	      faxLineBuffer [theFax. bufferP] = sampleValue;
	      theFax. bufferP = (theFax. bufferP + 1) % faxMode. samplesperLine;
	      if (theFax. linesRecognized > faxMode. nrLines - 10) {
	         checkBuffer [theFax. checkP] = sampleValue;
	         theFax. checkP ++;
	         if (theFax. checkP >= faxMode. samplesperLine) {
	            int upCrossings = checkFrequency (checkBuffer,
	                                              faxMode. samplesperLine,
	                                              faxMode. aptStopFreq,
	                                              false);
	            if (upCrossings > 0)
	               theFax. stoppers ++;
	            else
	               theFax. stoppers = 0;

	            if (theFax. stoppers >= 8) {
	               theFax. faxState = FAX_DONE;
	               break;
	            }

	            theFax. checkP = shiftBuffer (checkBuffer,
	                                  faxMode. samplesperLine / 2,
	                                  faxMode. samplesperLine);
	         }
	      }

	      if (theFax. bufferP == 0) {
	         if ((int32_t) (rawData. size ()) <=
	                                theFax. currentSampleIndex +
	                                              faxMode. samplesperLine) 
	            rawData. resize (rawData. size () + 1024 * 1024);

	         for (int i = 0; i < faxMode. samplesperLine; i ++)
	            rawData [theFax. currentSampleIndex ++] = faxLineBuffer [i];
	         processBuffer (faxLineBuffer,
	                        theFax. linesRecognized,
	                        faxMode. samplesperLine);
	         theFax. linesRecognized ++;
	         theFax. bufferP = 0;

	         show_lineno (theFax. linesRecognized);
	         if (theFax. linesRecognized > faxMode. nrLines + overflow. load ())
	            theFax. faxState = FAX_DONE;
	      }
	      break;

	   case FAX_DONE:
	      show_faxState (std::string ("FAX_DONE"));
	      if (saveContinuous) {
	         saveImage_auto ();
	      }
	      theFax. faxState	= WAITER;
	      toRead	= 10 * faxMode. samplesperLine;
	      break;

	   case WAITER:
	      toRead --;
	      if (saveContinuous && (toRead == 0))
	         theFax. faxState = APTSTART;
	      else
	      if (!saveContinuous && setCorrection) {
	         doCorrection ();
	         setCorrection = false;
	      }
	      if (!saveContinuous && saveSingle) {
	         saveImage_single();
	         saveSingle = false;
	      }
	      break;

	   default:		// cannot happen
	      theFax. faxState = APTSTART;
	}
}
//
static inline 
float	square (float f) {
	return f * f;
}

//	The start signal is a 300 Hz signal, we count the up-transitions
//	however, that is insufficient to discriminate, so we add a check
//	on the distances between successive up-transitions.
//	
//	The stop signal is a 450 Hz signal, we use a single function
//	for checking these frequencies
int	SDRunoPlugin_fax::checkFrequency (std::vector<int> &buffer,
	                                  int length, int Frequency, bool b) {
int	upCrossings = 0;
int	correctAmount	= (Frequency * length) / WORKING_RATE;
std::vector<int> crossings;

	for (int i = 1; i < length; i ++) {
	   if (realBlack (buffer [i - 1]) &&
	      realWhite (buffer [i])) {
	      crossings .push_back (i);
	      upCrossings ++;
	      i++;
	   }
	}

	show_aptLabel (upCrossings);
	if ((upCrossings < correctAmount - 5) ||
	    (upCrossings > correctAmount + 5)) {
	   return -1;
	}
//
//	we now know that the number of upcrossings is approx right
//	we compute the error between the measured distances between
//	successive upcrossings and the distance it should be for
//	a decent signal with frequency Frequency.
	float 	error	= 0;
	for (int i = 1; i < crossings. size (); i ++) {
	   int ff = crossings [i] - crossings [i - 1];
	   error += square (length / upCrossings - ff);
	}

	error = sqrt (error);
	return error / upCrossings < 2.0 ? upCrossings : -1;
}
//
int	SDRunoPlugin_fax::checkPhase	(std::vector<int> &buffer,
	                                   int index, float threshold) {
int	baseP	= findPhaseLine (buffer, 0, faxMode. samplesperLine, threshold);
	(void)index;
	if (baseP < 0)
	   return -1;
	show_aptLabel (baseP);
	if (!checkPhaseLine (buffer,
	                     baseP + 1 * faxMode. samplesperLine, threshold))
	   return -1;
	if (!checkPhaseLine (buffer,
	                     baseP + 2 * faxMode. samplesperLine, threshold))
	   return -1;
	if (!checkPhaseLine (buffer,
	                     baseP + 3 * faxMode. samplesperLine, threshold))
	   return -1;
//	if (!checkPhaseLine (buffer,
//		             baseP + 4 * faxMode. samplesperLine, threshold))
//	   return -1;
	return baseP;
}
//
//	A phaseLine starts with 2.5 percent white, then 95 percent black
//	and ending with 2.5 percent white
bool	SDRunoPlugin_fax::checkPhaseLine (std::vector<int> &buffer,
	                                  int index, float threshold) {
int	L1	= 2.5 * faxMode. samplesperLine / 100;
int	nrWhites	= 0;
int	nrBlacks	= 0;

	for (int i = 0; i < L1; i ++)
	   if (realWhite (buffer [index + i]) &&
	       realWhite (buffer [index + faxMode. samplesperLine - i - 1]))
	      nrWhites ++;

	if (nrWhites < threshold *  L1)
	   return false;

	for (int i = 0; i < faxMode. samplesperLine; i ++)
	   if (realBlack (buffer [index + i]))
	      nrBlacks ++;

	return nrBlacks > threshold * (0.95 * faxMode. samplesperLine);
}

int	SDRunoPlugin_fax::findPhaseLine	(std::vector<int> &buffer,
	                                   int ind, int end, float threshold) {
	for (int i = ind; i < end; i ++)
	   if (checkPhaseLine (buffer, i, threshold))
	      return i;
	return -1;
}

int	SDRunoPlugin_fax::shiftBuffer	(std::vector<int> &v,
	                                          int start, int end) {
	for (int i = start; i < end; i ++)
	   v [i - start] = v [i];
	return end - start;
}
//
//	A sample may contribute to more than one pixel. E.g., for a samplerate
//	of 12000, and an IOC of 576, there are 6.6 samples contributing.
//	We therefore look for the partial contribution of the first
//	and the last sample for a pixel
//	
void	SDRunoPlugin_fax::processBuffer	(std::vector<int> &buffer,
	                                          int currentRow,
	                                          int samplesLine) {
int	currentColumn	= 0;
int	pixelSamples	= 0;
float	pixelValue	= 0;

	for (int samplenum = 0; samplenum < samplesLine; samplenum ++) {
	   int x = buffer [samplenum];
	   float temp	= (float)faxMode. nrColumns / faxMode. samplesperLine;
	   int	columnforSample	=
	        floor ((float)samplenum / faxMode. samplesperLine * faxMode. nrColumns);
	   if (columnforSample == currentColumn) { // still dealing with the same pixel
	      if (temp * faxMode. nrColumns > currentColumn) {	// partial contribution
	         float part_0, part_1;
// part 0 is for this pixel
	         part_0 = temp * faxMode. nrColumns - currentColumn;
// and part_1 is for the next one
	         part_1		= (1 - (temp * faxMode. nrColumns - currentColumn));
	         pixelValue	+= part_0 * x;
	         pixelSamples 	+= part_0;
	         addPixeltoImage (pixelValue / pixelSamples,
	                               currentColumn, currentRow);
	         currentColumn ++;
	         pixelValue	= part_1 * x;
	         pixelSamples	= part_1;
	         continue;
	      }

	      pixelValue	+= x;
	      pixelSamples	++;
	      continue;
	   }
//
//	we expect here currentCol > currentColumn
	   if (pixelSamples > 0) 	// simple "assertion"
	      addPixeltoImage (pixelValue / pixelSamples,
	                         currentColumn, currentRow);

	   currentColumn	= columnforSample;
	   pixelValue		= x;
	   pixelSamples		= 1;
	}
}

void	SDRunoPlugin_fax::processLine (std::vector<float> &result,
	                               std::vector<float> &samples,
	                               int columns, int samplesperLine) {
int currentColumn	= 0;
float pixelSamples	= 0;
float pixelValue	= 0;

	for (int sampleNum = 0; sampleNum < samplesperLine; sampleNum ++) {
	   float x = samples [sampleNum];
	   float temp = (float)columns / samplesperLine;
	   int columnforSample = 
	      floor ((float)sampleNum / samplesperLine * columns);

	   if (columnforSample == currentColumn) { // the same pixel
	      if (temp * faxMode. nrColumns > currentColumn) { //partial contribution
	         float part_0, part_1;
// part 0 is for this pixel
	         part_0 = temp * columns - currentColumn;
// and part_1 is for the next one
	         part_1         = (1 - (temp * columns - currentColumn));
	         pixelValue     += part_0 * x;
	         pixelSamples   += part_0;
	             result [currentColumn] = pixelValue / pixelSamples;
	         currentColumn ++;
	         pixelValue     = part_1 * x;
	         pixelSamples   = part_1;
	         continue;
	      }
	   }
//
//      we expect here currentCol > currentColumn
	   if (pixelSamples > 0)        // simple "assertion"
	          result [currentColumn] = pixelValue / pixelSamples;
	   currentColumn        = columnforSample;
	   pixelValue           = x;
	   pixelSamples         = 1;
	}
}
//
//
void	SDRunoPlugin_fax::addPixeltoImage (float val,
	                                     int32_t col, int32_t row) {
int32_t realRow = faxMode. faxColor == FAX_COLOR ? row / 3 : row;

	if (row  * faxMode. nrColumns + col >= pixelStore. size ())
	   pixelStore. resize (pixelStore. size () + 10 * faxMode. nrColumns);
	pixelStore [row * faxMode. nrColumns + col] = val;
	
	if (theFax. lastRow < row) {
	   theFax. flipper --;
	   if (theFax. flipper <= 0) {
	      drawPicture (theFax. lastRow);
	      theFax. flipper = 10;
	   }
	   theFax. lastRow	= row;
	}
}
//
faxParams *SDRunoPlugin_fax::getFaxParams	(const std::string &s) {
int16_t	i;

	for (i = 0; _faxParams [i].Name != NULL; i ++)
	   if (s == _faxParams [i]. Name)
	      return &_faxParams [i];
	return NULL;
}

///*
// *	we add the demodulated value (x) to the
// *	current pixel, so first we find out
// *	the position of the current pixel
// *
// *	Number of samples per line =
// *		theRate  * 60.0 / lpm
// *	samplenumber in currentline =
// *		fmod (currentSampleIndex, theRate * 60.0 / lpm)
// *	position of sample in current column =
// *	       samplenumber in current line / number of samples per Line * x
// *
// *


/////////////////////////////////////////////////////////////////////////
//	Interface functions
//
//      coming from the GUI
void	SDRunoPlugin_fax::fax_setIOC	(const std::string &s) {
	this	-> selected_IOC = s;
	this	-> resetFlag = true;
}


void	SDRunoPlugin_fax::fax_setMode	(const std::string &s) {
	faxMode. demodMode = s == "AM" ? FAX_AM : FAX_FM;
}

void	SDRunoPlugin_fax::fax_setPhase	(const std::string &s) {
	faxMode. phaseInvers	= s == "invere";
}

void	SDRunoPlugin_fax::fax_setColor	(const std::string &s) {
	if (s == "BW")
	   faxMode. faxColor = FAX_BLACKWHITE;
	else
	if (s == "COLOR")
	   faxMode. faxColor = FAX_COLOR;
	else
	   faxMode. faxColor = FAX_GRAY;
}

void	SDRunoPlugin_fax::fax_setDeviation	(const std::string &s) {
	if (s == "1900-400") {
	   carrier	= 0;
	   deviation	= 400;
	}
	else {
	   carrier	= 0;
	   deviation	= 450;
	}
}

void	SDRunoPlugin_fax::handle_resetButton	() {
	resetFlag. store (true);
}
void	SDRunoPlugin_fax::handle_cheatButton() {
	cheatFlag. store (true);
//	if (theFax. faxState == SYNCED) {
//	   theFax. faxState = FAX_DONE;
//	}
//	else {
//	   theFax. bufferP 		= 0;
//	   theFax. checkP 		= 0;
//	   theFax. linesRecognized 	= 0;
//	   theFax. alarmCount		= 0;
//	   theFax. currentSampleIndex	= 0;
//	   theFax. lastRow 		= 0;
//	   theFax. stoppers 		= 0;
//	   theFax. sampleOffset		= 0;
//	   theFax. faxState 		= SYNCED;
//	   clearScreen		();
//	   show_faxState	("ON SYNC");
//	}
}

void	SDRunoPlugin_fax::handle_saveContinuous	() {
	saveContinuous = !saveContinuous;
	show_savingLabel (saveContinuous ? "saving" : "Save Cont");
}

void	SDRunoPlugin_fax::handle_saveSingle	() {
	if (saveContinuous)
	   return;
	saveSingle = true;
}

void	SDRunoPlugin_fax::set_overflow	(int n) {
	overflow. store (n);
}
//
void	SDRunoPlugin_fax::show_faxState	(const std::string &s) {
	m_form. show_faxState (s);
}

void	SDRunoPlugin_fax::show_lineno	(int n) {
	m_form. show_lineno (n);
}

void	SDRunoPlugin_fax::show_savingLabel	(const std::string &s) {
	m_form. show_savingLabel (s);
}

void	SDRunoPlugin_fax::show_aptLabel	(int n) {
	m_form. show_aptLabel (n);
}

static inline
bool	isValid (char c) {
	if (('0' <= c) && (c <= '9'))
	   return true;
	if (('a' <= c) && (c <= 'z'))
		return true;
	if (('A' <= c) && (c <= 'Z'))
		return true;
	return false;
}

std::string getFileName () {
std::time_t result = std::time (nullptr);
std::string theTime;
char* home = getenv ("HOMEPATH");
char * tt = std::asctime (std::localtime (&result));
	if (tt == 0)
	   theTime = "no-time";
	else {
	   for (int i = 0; tt [i] != 0; i ++)
	      if (isValid (tt [i]))
	        theTime. push_back (tt [i]);
	      else
	        theTime. push_back ('-');
	}
	return std::string(home) + "\\wFax-" + theTime + ".bmp";
}

void	SDRunoPlugin_fax::drawPicture (int lastRow) {
std::vector<float> line  (faxMode. nrColumns);
std::vector<float> out_1   (faxWidth + 10);
std::vector<float> out_2   (faxWidth + 10);

	for (int i = 0; i < lastRow / 2 ; i ++) {
	   for (int j = 0; j < faxMode.  nrColumns; j ++)
	      line [j] = pixelStore [2 * i *  faxMode. nrColumns + j];
	   processLine (out_1, line, faxWidth, faxMode. nrColumns);

	   for (int j = 0; j < faxMode. nrColumns; j ++)
	      line [j] = pixelStore [(2 * i + 1) * faxMode. nrColumns + j];
	   processLine (out_2, line, faxWidth, faxMode. nrColumns);
	  
	   for (int j = 0; j < faxWidth; j ++)
	      out_1 [j] = Minimum (out_1 [j], out_2 [j]);
	   if (i < faxHeight)
	      m_form. drawLine (out_1, i);
	}
	m_form. updateImage ();
}

void	SDRunoPlugin_fax::clearScreen () {
std::vector<float> theRow (faxWidth);

	for (int i = 0; i < faxWidth; i ++)
	   theRow [i] = 255;
	for (int row = 0; row < faxHeight; row++)
	   m_form. drawLine (theRow, row);
	m_form. updateImage ();
}

void	SDRunoPlugin_fax::set_correctionFactor (int f) {
	theFax. sampleOffset = f;
}

void	SDRunoPlugin_fax::regenerate() {
	if ((theFax. faxState != WAITER) && (theFax. faxState != FAX_DONE))
	   return;
	setCorrection = true;
}

void	SDRunoPlugin_fax::doCorrection () {
int sampleTeller	= 0;
std::vector<int> lineBuffer;
int	lineno		= 0;
int	lineSamples	= faxMode. samplesperLine + theFax. sampleOffset;
int	maxi		= lineSamples > faxMode. samplesperLine ?
	                        lineSamples : faxMode. samplesperLine;

	if ((theFax. faxState != WAITER) && (theFax. faxState != FAX_DONE))
	   return;
	clearScreen ();

	lineBuffer. resize (maxi);
	correcting. store(true);
	while (sampleTeller < theFax. currentSampleIndex + maxi) {
	   if ((lineno % 10) != 0) {
	      for (int i = 0; i < faxMode. samplesperLine; i ++) 
	         lineBuffer [i] = rawData [sampleTeller ++];
	      processBuffer (lineBuffer, lineno, faxMode. samplesperLine);
	   }
	   else
	      for (int i = 0; i < lineSamples; i ++) {
	         lineBuffer [i] = rawData [sampleTeller ++];
	      processBuffer (lineBuffer, lineno, lineSamples);
	   }
	   lineno ++;
	}
	correcting. store (false);
}

void	SDRunoPlugin_fax::saveImage_single	() {
	if ((theFax. faxState != WAITER) && (theFax. faxState != FAX_DONE))
	   return;
	if (saveContinuous)
	   return;
	char* home = getenv("HOMEPATH");
	nana::filebox fb (0, false);
	fb.add_filter ("bitmap file", "*.bmp");
	fb.add_filter ("All Files", "*.*");
	fb. init_path (home);
	auto files = fb();
	if (!files. empty ()) {
	   std::string fileName = files.front().string();
	   nana::paint::graphics graph (nana::size (faxMode. nrColumns,
	                                            faxMode. nrLines));
	   for (int row = 0; row < faxMode.  nrLines; row++) {
	      for (int column = 0; column < faxMode. nrColumns; column++) {
	         if (pixelStore [row * faxMode. nrColumns + column] > 96)
	            graph. set_pixel (column, row, nana::colors::white);
	         else
	            graph. set_pixel (column, row, nana::colors::black);
	      }
	   }
	   show_faxState (fileName);
	   graph. save_as_file (fileName. c_str ());
	}
}

void	SDRunoPlugin_fax::saveImage_auto	() {
nana::paint::graphics graph (nana::size (faxMode. nrColumns,
	                                 faxMode. nrLines));

	for (int row = 0; row < faxMode. nrLines; row++) {
	   for (int column = 0; column < faxMode. nrColumns; column++) {
	      if (pixelStore [row * faxMode. nrColumns + column] > 96)
	         graph. set_pixel (column, row, nana::colors::white);
	      else
	         graph. set_pixel (column, row, nana::colors::black);
	   }
	}
	std::string fileName = getFileName ();
	show_faxState (fileName);
	graph. save_as_file (fileName. c_str ());
}

int	SDRunoPlugin_fax::demodulate	(std::complex<float> z) {
float	res;
static std::complex<float> prevSample = std::complex<float> (0, 0);

        z               = cdiv (z, abs (z));
        if (faxMode. demodMode == FAX_AM)
           return abs (z) * 255.0;

        res	= arg (conj (prevSample) * z) / (2 * M_PI) * WORKING_RATE;
        res	= clamp (res, - this -> deviation,
	                      + this -> deviation);
        prevSample      = z;
        return (int16_t)(res / deviation * 128 + 127);
}

