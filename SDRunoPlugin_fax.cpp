#include	<sstream>
#include	<unoevent.h>
#include	<iunoplugincontroller.h>
#include	<vector>
#include	<sstream>
#include	<chrono>
#include        <Windows.h>
#include	<stdio.h>
#include	<stdlib.h>

#include "SDRunoPlugin_fax.h"
#include "SDRunoPlugin_faxUi.h"
//
//	fax specfics

#include	"fax-bandfilter.h"
#include	"fax-shifter.h"
#include	"utilities.h"
#include        "up-filter.h"
//
////	for the dump-filename
#include	<time.h>
#include	<ctime>
#define	FAX_IF 0

//static FILE	*dumpFile	= nullptr;
//#define __TESTING__ 1
faxParams _faxParams [] = {
	{"Wefax288", 288, 675, 450, false, 120, 600},
	{"Wefax576", 576, 300, 450, false, 120, 1200},
	{"HamColor", 204, 200, 450, true,  360, 0},	// not visible
	{"Ham288b",  240, 675, 450, false, 240, 0},	// not visible
	{"Color240", 288, 200, 450, true,  240, 0},	// not visible
	{"FAX480",   204, 500, 450, false, 480, 0},	// not visible
	{NULL,        -1,  -1,  -1, false,  -1, 0}	
};

#ifdef	__TESTING__
static std::complex<float> oscillatorTable [12000];
static int testPhase	= 0;
#endif

#define  _USE_MATH_DEFINES
#include        <math.h>

static inline
float Minimum(float x, float y) {
	return x < y ? x : y;
}

	SDRunoPlugin_fax::
	              SDRunoPlugin_fax (IUnoPluginController& controller):
	                                   IUnoPlugin	(controller),
	                                   m_form	(*this, controller),
	                                   m_worker	(nullptr),
	                                   inputBuffer	(64 * 32768),
	                                   theMixer	(INRATE),
	                                   passbandFilter (11,
	                                                   -2000,
	                                                   +2000,
	                                                   INRATE),
	                                   theDecimator (DECIMATOR),
	                                   localMixer (WORKING_RATE),
	                                   faxLineBuffer (600),
	                                   faxAudioBuffer (16 * 32768),
	                                   faxToneBuffer (192) {
	m_controller            = &controller;
	running. store (false);

//      we want to "work" with a rate of 12000, and since we arrive
//      from IN_RATE we first decimate and filter to 12500 and then
//      interpolate for the rest
	for (int i = 0; i < WORKING_RATE / 100; i ++) {
	   float inVal  = float (INTERM_RATE / 100);
	   mapTable_int [i]     = int (floor (i * (inVal / (WORKING_RATE / 100))));
	   mapTable_float [i]   = i * (inVal / (WORKING_RATE / 100)) - mapTable_int [i];
	}
	convIndex       = 0;
	convBuffer. resize (INTERM_RATE / 100 + 1);

	
	   overflow	= 60;

#ifdef	__TESTING__
	for (int i = 0; i < 12000; i ++)
	   oscillatorTable [i] =
	      std::complex<float> (cos ((float)i / 12000 * 2 * M_PI),
	                           sin ((float)i / 12000 * 2 * M_PI));
	testPhase	= 0;
#endif
	faxTonePhase   = 0;
	m_controller    -> RegisterStreamProcessor (0, this);
	m_controller    -> RegisterAudioProcessor (0, this);
	selectedFrequency
	                = m_controller -> GetVfoFrequency (0);
	centerFrequency	= m_controller -> GetCenterFrequency (0);
	faxAudioRate	= m_controller -> GetAudioSampleRate (0);
	Raw_Rate		= m_controller -> GetSampleRate (0);

	faxError         = false;
	if ((Raw_Rate != 2000000 / 32) || (faxAudioRate != 48000)) {
	   show_faxState ("Please set input rate 2000000 / 32 and audiorate to 48000");
	   faxError      = true;
	}

	faxToneBuffer. resize (faxAudioRate);
	for (int i = 0; i < faxAudioRate; i ++) {
	   float term = (float)i / faxAudioRate * 2 * M_PI;
	   faxToneBuffer [i] = std::complex<float> (cos (term), sin (term));
	}

	audioFilter	= new upFilter (25, WORKING_RATE, faxAudioRate);
	faxTonePhase	= 0;
	carrier		= FAX_IF;
	deviation	= 400;		// default
	setup_faxDecoder	("Wefax576");
	faxColor	= FAX_BLACKWHITE;
	sampleOffset	= 0;
	correcting.store  (false);
	setCorrection = false;
	saveContinuous	= false;
	saveSingle	= false;
	running. store (false);
//
//	we draw the map on an label with a size of 900 x 700
	faxContainer = new drawing (*(m_form. getArea()));
	faxContainer -> draw ([&](paint::graphics& graph) {
	        for (int i = 0; i < lastRow / 2 ; i ++)
		   for (int j = 0; j < numberofColumns; j++) {
			   float res =
			   Minimum (pixelStore[2 * i * numberofColumns + j],
	                        pixelStore [2 * (i + 1) * numberofColumns + j]);

	           if ((i < 700) && (j < 900))
	                 graph.set_pixel(j, i, res >= 128 ?
	                                          nana::colors::white :
	                                          nana::colors::black);
	           }
		});

	m_worker = new std::thread (&SDRunoPlugin_fax::WorkerFunction, this);
}

	SDRunoPlugin_fax::~SDRunoPlugin_fax () {	
	running.store(false);
	m_worker 	-> join();
	m_controller    -> UnregisterStreamProcessor (0, this);
//	m_controller    -> UnregisterAudioProcessor (0, this);
	faxContainer	-> clear();
	delete	        faxContainer;
	delete	        m_worker;
	delete	        myDemodulator;
	delete	        audioFilter;
	delete	        faxAverager;
	delete	        faxLowPass;
}

void	SDRunoPlugin_fax::
	         StreamProcessorProcess (channel_t channel,
	                                 Complex* buffer,
	                                 int	length,
	                                 bool& modified) {
	if (running. load () && !faxError && !correcting. load ()) {
	   inputBuffer. putDataIntoBuffer (buffer, length);
	}
	modified = false;
}

void	SDRunoPlugin_fax::AudioProcessorProcess (channel_t channel,
	                                         float* buffer,
	                                         int length,
	                                         bool& modified) {
	if (faxAudioBuffer. GetRingBufferReadAvailable () >= length * 2) {
	   faxAudioBuffer. getDataFromBuffer (buffer, length * 2);
	   modified = true;
	}
	else
	   modified = false;
}

void	SDRunoPlugin_fax::HandleEvent (const UnoEvent& ev) {
	switch (ev. GetType ()) {
	   case UnoEvent::FrequencyChanged:
	      selectedFrequency =
	              m_controller ->GetVfoFrequency (ev. GetChannel ());
	      centerFrequency = m_controller -> GetCenterFrequency(0);
	      locker. lock ();
	      passbandFilter.
	             update (selectedFrequency - centerFrequency, 2000);
	      locker. unlock ();
	      break;

	   case UnoEvent::CenterFrequencyChanged:
	      break;

	   default:
	      m_form. HandleEvent (ev);
	      break;
	}
}

#define	BUFFER_SIZE 4096
Complex buffer[BUFFER_SIZE];
void	SDRunoPlugin_fax::WorkerFunction () {

	running. store (true);
	while (running. load ()) {
	   while (running. load () &&
	              (inputBuffer. GetRingBufferReadAvailable () < BUFFER_SIZE))
	      Sleep (1);
	   if (!running. load ())
	      break;

	   (void)inputBuffer. getDataFromBuffer (buffer, BUFFER_SIZE);
	   int theOffset = centerFrequency - selectedFrequency;
	   for (int i = 0; i < BUFFER_SIZE; i++) {
	      std::complex<float> sample =
	                std::complex<float>(buffer [i]. real, buffer [i]. imag);
	      locker.lock ();
	      sample   = passbandFilter. Pass (sample);
	      locker.unlock ();
	      sample   = theMixer. do_shift (sample, -theOffset);
	      if (theDecimator. Pass (sample, &sample))
	         process (sample);
	   }  
	}

	show_faxState ("going down");
	Sleep(1000);
}

static inline
std::complex<float> cmul (std::complex<float> x, float y) {
	return std::complex<float>(real(x) * y, imag(x) * y);
}

int     SDRunoPlugin_fax::resample       (std::complex<float> in,
	                                      std::complex<float> *out) {
	convBuffer [convIndex ++] = in;
	if (convIndex >= convBuffer. size ()) {
	   for (int i = 0; i < WORKING_RATE / 100; i ++) {
	      int16_t  inpBase       = mapTable_int [i];
	      float    inpRatio      = mapTable_float [i];
	      out [i]       = cmul (convBuffer [inpBase + 1], inpRatio) +
	                          cmul (convBuffer [inpBase], 1 - inpRatio);
	   }
	   convBuffer [0]       = convBuffer [convBuffer. size () - 1];
	   convIndex    = 1;
	   return WORKING_RATE / 100;
	}
	return -1;
}

void	SDRunoPlugin_fax::setup_faxDecoder	(std::string IOC_name) {
std::string h;
int	k;
	faxLowPass		= new LowPassFIR (FILTER_DEFAULT,
	                                       //  deviation + 50,
		                                   500,
	                                           WORKING_RATE);
	faxAverager		= new faxAverage (20);
	myDemodulator		= new faxDemodulator (FAX_FM,
	                                              WORKING_RATE,
	                                              deviation);
//	OK, we know now
	faxParams *myfaxParameters 	= getFaxParams (IOC_name);
	lpm			= myfaxParameters -> lpm;
	
	samplesperLine		= WORKING_RATE * 60 / lpm;
	faxLineBuffer. resize (5 * samplesperLine);
	checkBuffer. resize (samplesperLine);
	fax_IOC			= myfaxParameters -> IOC;
	numberofColumns		= M_PI * fax_IOC;
	if (numberofColumns > 900)
	   numberofColumns 	= 900;
	nrLines			= myfaxParameters -> nrLines;
	faxColor		= myfaxParameters -> color ?
	                                    FAX_COLOR: FAX_BLACKWHITE;
	carrier			= FAX_IF;	// default
	phaseInvers		= false;
	lastRow			= 0;	// will change	
	pixelStore. resize (numberofColumns * nrLines);
	rawData. resize (1024 * 1024);
//
	faxState		= APTSTART;
	apt_upCrossings		= 0;
	currentSampleIndex	= 0;
	aptStartFreq		= myfaxParameters -> aptStart;
	aptStopFreq = myfaxParameters->aptStop;
	show_faxState (std::string ("APTSTART"));
}
	   
static inline
bool	realWhite (int16_t x) {
	return x > 229;
}

static inline
bool	realBlack(int16_t x) {
	return x < 25;
}

static inline
bool	isWhite (int16_t x) {
	return x >= 128;
}
//
//	as always, we "process" one sample at the time.
//
void    SDRunoPlugin_fax::process (std::complex<float> z) {
std::complex<float> out [256];  // IN_RATE / DECIMATOR
int     cnt;

	cnt = resample (z, out);
	if (cnt < 0)
	   return;

	for (int i = 0; i < cnt; i++) {
	   processSample (out[i]);
	}
}

void	SDRunoPlugin_fax::processSample (std::complex<float> z) {
int sampleValue;
int	baseP;
std::vector<std::complex<float>> tone (faxAudioRate / WORKING_RATE);

	locker. lock ();
	z	= localMixer. do_shift (z, carrier);
	z	= faxLowPass    -> Pass (z);
#ifdef	__TESTING__
	sampleValue	= real(oscillatorTable[testPhase]) * 128 + 128;
	testPhase	= (testPhase + 300) % WORKING_RATE;
#else
	sampleValue	= myDemodulator -> demodulate (z);
#endif
	locker. unlock ();

	audioFilter -> Filter (cmul (z, 20), tone. data ());
	for (int i = 0; i < tone. size (); i ++) {
	   tone [i] *= faxToneBuffer [faxTonePhase];
	   faxTonePhase = (faxTonePhase + 801) % faxAudioRate; 
	} 
	faxAudioBuffer. putDataIntoBuffer (tone. data (), tone. size () * 2);

	if (phaseInvers)
	   sampleValue = 256 - sampleValue;
	if (faxColor == FAX_BLACKWHITE)
	   sampleValue = isWhite (sampleValue) ? 255 : 0;
	
	
	switch (faxState) {
	   case	APTSTART:
	      show_faxState	("APTSTART");
	      clearScreen	();
	      bufferP		= 0;
	      lastRow		= 0;
	      linesRecognized	= 0;
	      checkP		= 0;
	      faxLineBuffer [bufferP] = sampleValue;
	      bufferP ++;
	      faxState		= WAITING_FOR_START;
	      currentSampleIndex	= 0;
	      break;

	   case WAITING_FOR_START:
	      faxLineBuffer [bufferP] = sampleValue;
	      bufferP ++;
	      if (bufferP >= samplesperLine) {
	         int upCrossings = checkFrequency (faxLineBuffer,
	                                           samplesperLine,	
	                                           aptStartFreq, true);
	         if (upCrossings > 0) {
	            faxState = START_RECOGNIZED;
	            toRead = 6 * samplesperLine;
	            bufferP = 0;
	         }
	         else {
	            for (int i = samplesperLine / 10; i < samplesperLine; i ++)
	               faxLineBuffer [i - samplesperLine / 10] =
	                                               faxLineBuffer [i];
                    bufferP -= samplesperLine / 10;
	         }
	      }
	      break;

	   case START_RECOGNIZED:
	      toRead --;
	      if (toRead <= 0) {
	         faxState	= WAITING_FOR_PHASE;
	         show_faxState ("PHASING");
	         alarmCount	= 0;
	         bufferP 	= 0;
	      }
	      break;

	   case WAITING_FOR_PHASE:
	      faxLineBuffer [bufferP] = sampleValue;
	      bufferP ++;
	      if (bufferP == faxLineBuffer. size () - 2) {
	         faxState = READ_PHASE;
	      }
	      break;

	   case READ_PHASE:
	      faxLineBuffer [bufferP] = sampleValue;
	      baseP = checkPhase (faxLineBuffer, 0, 0.90);
	      if (baseP >= 0) {
	         for (int i = baseP; i < samplesperLine; i ++)
	            faxLineBuffer [i - baseP] = faxLineBuffer [i];
                 bufferP        = samplesperLine - baseP;
	         currentSampleIndex = 0;
                 checkP         = 0;
	         faxState	= SYNCED;
	         show_faxState ("ON SYNC");
	         currentSampleIndex	= 0;
	         stoppers	= 0;
	         linesRecognized = 0;
	      }
	      else {
	         for (int i = samplesperLine; i < faxLineBuffer. size (); i ++)
	            faxLineBuffer [i - samplesperLine] = faxLineBuffer [i];
	         bufferP = faxLineBuffer. size () - samplesperLine - 1;
	         alarmCount ++;
	         show_lineno (alarmCount);
	         if (alarmCount >= 15)
	            faxState = APTSTART;
	         else
	            faxState = WAITING_FOR_PHASE;
	      }
	      break;

	   case SYNCED:
	      faxLineBuffer [bufferP] = sampleValue;
	      bufferP = (bufferP + 1) % samplesperLine;
	      if (linesRecognized > nrLines + 20) {
	         checkBuffer [checkP] = sampleValue;
	         checkP ++;
	         if (checkP >= samplesperLine) {
	            int upCrossings = checkFrequency (checkBuffer,
	                                              samplesperLine,
	                                              aptStopFreq, false);
	            if (upCrossings > 0)
	               stoppers ++;
	            else
	               stoppers = 0;
	            if (stoppers >= 8) {
	               faxState = FAX_DONE;
	               break;
	            }
	            checkP = shiftBuffer (checkBuffer,
	                                  samplesperLine / 10, samplesperLine);
	         }
	      }
	      if (bufferP == 0) {
	         if ((int32_t) (rawData. size ()) <= currentSampleIndex +
	                                              samplesperLine) 
	            rawData. resize (rawData. size () + 1024 * 1024);
	         for (int i = 0; i < samplesperLine; i ++)
	            rawData [currentSampleIndex ++]	= faxLineBuffer [i];
	         processBuffer (faxLineBuffer, linesRecognized, samplesperLine);
	         linesRecognized ++;
	         bufferP = 0;
	         show_lineno (linesRecognized);
	         if (linesRecognized > nrLines + overflow)
	            faxState = FAX_DONE;
	      }
	      break;

	   case FAX_DONE:
	      show_faxState (std::string ("FAX_DONE"));
	      if (saveContinuous) {
	         saveImage_auto ();
	      }
	      faxState	= WAITER;
	      toRead	= 10 * samplesperLine;
	      break;

	   case WAITER:
	      toRead --;
	      if (saveContinuous && (toRead == 0))
	         faxState = APTSTART;
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
	      faxState = APTSTART;
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
	if ((upCrossings < correctAmount - 3) ||
	    (upCrossings > correctAmount + 3)) {
	   return -1;
	}
//
//	we now know that the number of upcrossings is app right
//	we compute the error between the measures distances between
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
int	baseP	= findPhaseLine (buffer, 0, samplesperLine, threshold);
	(void)index;
	if (baseP < 0)
	   return -1;
	show_aptLabel (baseP);
	if (!checkPhaseLine (buffer, baseP + 1 * samplesperLine, threshold))
	   return -1;
	if (!checkPhaseLine (buffer, baseP + 2 * samplesperLine, threshold))
	   return -1;
	if (!checkPhaseLine (buffer, baseP + 3 * samplesperLine, threshold))
	   return -1;
//	if (!checkPhaseLine (buffer, baseP + 4 * samplesperLine, threshold))
//	   return -1;
	return baseP;
}
//
//	A phaseLine starts with 2.5 percent white, then 95 percent black
//	and ending with 2.5 percent white
bool	SDRunoPlugin_fax::checkPhaseLine (std::vector<int> &buffer,
	                                     int index, float threshold) {
int	L1	= 2.5 * samplesperLine / 100;
int	nrWhites	= 0;
int	nrBlacks	= 0;

	for (int i = 0; i < L1; i ++)
	   if (realWhite (buffer [index + i]) &&
	       realWhite (buffer [index + samplesperLine - i - 1]))
	      nrWhites ++;

	if (nrWhites < threshold *  L1)
	   return false;

	for (int i = 0; i < samplesperLine; i ++)
	   if (realBlack (buffer [index + i]))
	      nrBlacks ++;

	return nrBlacks > threshold * (0.95 * samplesperLine);
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
	   float temp	= (float)numberofColumns / samplesLine;
	   int	columnforSample	=
	        floor ((float)samplenum / samplesLine * numberofColumns);
	   if (columnforSample == currentColumn) { // still dealing with the same pixel
	      if (temp * numberofColumns > currentColumn) {	// partial contribution
	         float part_0, part_1;
// part 0 is for this pixel
	         part_0 = temp * numberofColumns - currentColumn;
// and part_1 is for the next one
	         part_1		= (1 - (temp * numberofColumns - currentColumn));
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
//
//
static int flipper = 10;
void	SDRunoPlugin_fax::addPixeltoImage (float val,
	                                     int32_t col, int32_t row) {
int32_t realRow = faxColor == FAX_COLOR ? row / 3 : row;

	if (row  * numberofColumns + col >= pixelStore. size ())
	   pixelStore. resize (pixelStore. size () + 10 * numberofColumns);
	pixelStore [row * numberofColumns + col] = val;
	
	if (lastRow < row) {
	   flipper --;
	   if (flipper <= 0) {
	      faxContainer -> update ();
	      flipper = 10;
	   }
	   lastRow		= row;
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
faxParams	*myfaxParameters	= getFaxParams (s);
	locker. lock ();
	fax_IOC			= myfaxParameters -> IOC;
	numberofColumns		= M_PI * fax_IOC;
	aptStartFreq		= myfaxParameters -> aptStart;
	aptStopFreq		= myfaxParameters -> aptStop;
	lpm			= myfaxParameters -> lpm;
	samplesperLine		= WORKING_RATE * 60 / lpm;
	lastRow			= 0;	// will change
	faxState		= APTSTART;
	rawData. resize (1024 * 1024);
	locker. unlock ();
}

void	SDRunoPlugin_fax::fax_setMode	(const std::string &s) {
	myDemodulator -> setMode (s == "AM" ? FAX_AM : FAX_FM);
}

void	SDRunoPlugin_fax::fax_setPhase	(const std::string &s) {
	phaseInvers	= s == "invere";
}

void	SDRunoPlugin_fax::fax_setColor	(const std::string &s) {
	if (s == "BW")
	   faxColor = FAX_BLACKWHITE;
	else
	if (s == "COLOR")
	   faxColor = FAX_COLOR;
	else
	   faxColor = FAX_GRAY;

}

void	SDRunoPlugin_fax::fax_setDeviation	(const std::string &s) {
int	newIF;
int	newDeviation;

	if (s == "1900-400") {
		newIF = 0;
	   newDeviation	= 400;
	}
	else {
		newIF = 0;
	   newDeviation	= 450;
	}
	if (deviation == newDeviation)
	   return;

	locker. lock ();
	deviation		= newDeviation;
	carrier			= newIF;
	delete faxLowPass;
	faxLowPass              = new LowPassFIR (FILTER_DEFAULT,
	                                          deviation + 50,
	                                          WORKING_RATE);
	delete myDemodulator;
	myDemodulator           = new faxDemodulator (FAX_FM,
	                                              WORKING_RATE,
	                                              deviation);
	locker. unlock ();
}

void	SDRunoPlugin_fax::handle_resetButton	() {
	faxState		= APTSTART;
	show_faxState (std::string ("APTSTART"));
	apt_upCrossings		= 0;
	linesRecognized		= 0;
	saveContinuous		= false;
	saveSingle		= false;
	show_savingLabel  ("Save Cont");
	rawData. resize (1024 * 1024);
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

void	SDRunoPlugin_fax::handle_cheatButton() {
	if (faxState == SYNCED) {
		faxState = FAX_DONE;
	}
	else {
	   clearScreen();
	   show_faxState("ON SYNC");
	   bufferP = 0;
	   lastRow = 0;
	   linesRecognized = 0;
	   checkP = 0;
	   faxState = SYNCED;
	   stoppers = 0;
	}
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
char* home = getenv("HOMEPATH");
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

void	SDRunoPlugin_fax::clearScreen () {
	for (int row = 0; row < nrLines; row++)
	   for (int column = 0; column < numberofColumns; column++)
	      pixelStore [row * numberofColumns + column] = 255;
	faxContainer -> update();
	lastRow = 0;
}

void	SDRunoPlugin_fax::set_correctionFactor (int f) {
	sampleOffset = f;
}

void	SDRunoPlugin_fax::regenerate() {
	if ((faxState != WAITER) && (faxState != FAX_DONE))
	   return;
	setCorrection = true;
}

void	SDRunoPlugin_fax::doCorrection () {
int sampleTeller	= 0;
std::vector<int> lineBuffer;
int	lineno		= 0;
int	lineSamples = samplesperLine + sampleOffset;
int maxi = lineSamples > samplesperLine ? lineSamples : samplesperLine;

	if ((faxState != WAITER) && (faxState != FAX_DONE))
	   return;
	clearScreen ();

	lineBuffer.resize (maxi);
	correcting. store(true);
	while (sampleTeller < currentSampleIndex + maxi) {
	   if ((lineno % 10) != 0) {
	      for (int i = 0; i < samplesperLine; i ++) 
	         lineBuffer [i] = rawData [sampleTeller ++];
	      processBuffer (lineBuffer, lineno, samplesperLine);
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
	if ((faxState != WAITER) && (faxState != FAX_DONE))
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
	   nana::paint::graphics graph (nana::size (numberofColumns, nrLines));
	   for (int row = 0; row < nrLines / 2; row++) {
	      for (int column = 0; column < numberofColumns; column++) {
	         if (pixelStore [2 * row * numberofColumns + column] > 96)
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
nana::paint::graphics graph (nana::size (numberofColumns, nrLines));

	for (int row = 0; row < nrLines / 2; row++) {
	   for (int column = 0; column < numberofColumns; column++) {
	      if (pixelStore [2 * row * numberofColumns + column] > 96)
	         graph. set_pixel (column, row, nana::colors::white);
	      else
	         graph. set_pixel (column, row, nana::colors::black);
	   }
	}
	std::string fileName = getFileName ();
	show_faxState (fileName);
	graph. save_as_file (fileName. c_str ());
}

