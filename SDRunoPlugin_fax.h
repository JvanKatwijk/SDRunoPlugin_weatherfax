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
#pragma once

#include	<thread>
#include	<mutex>
#include	<atomic>
#include	<iunoplugincontroller.h>
#include	<iunoplugin.h>
#include	<iunostreamobserver.h>
#include	<iunoaudioobserver.h>
#include	<iunoaudioprocessor.h>
#include	<iunostreamobserver.h>
#include	<iunoannotator.h>
#include	<nana/gui.hpp>
#include	"SDRunoPlugin_faxUi.h"

//      for the payload we have
class		upFilter;
#include        "ringbuffer.h"
#include        <stdint.h>
#include	"fax-shifter.h"
#include        "decimator-25.h"
#include	"fax-bandfilter.h"
#include	"lowpassfilter.h"
#include	"fax-params.h"

#define         INRATE          192000
#define         WORKING_RATE    12000
#define	        FILTER_DEFAULT	21

using namespace nana;
/*
 *      states:
 *      we look at the bits
 */
#define APTSTART                0001
#define WAITING_FOR_START       0002
#define START_RECOGNIZED        0004
#define WAITING_FOR_PHASE       0010
#define READ_PHASE              0020
#define SYNCED                  0040
#define FAX_DONE                0100
#define WAITER                  0200

#define	FAX_AM			0100
#define	FAX_FM		        0101

class SDRunoPlugin_fax : public IUnoPlugin,
	                             public IUnoStreamProcessor,
	                             public IUnoAudioProcessor {

public:
	
	        SDRunoPlugin_fax	(IUnoPluginController& controller);
	virtual ~SDRunoPlugin_fax	();

	virtual const char* GetPluginName() const override { return "SDRuno weatherfax"; }

	// IUnoPlugin
	virtual
	void	HandleEvent(const UnoEvent& ev) override;
	virtual
	void    StreamProcessorProcess (channel_t channel,
	                                Complex *buffer,
	                                int length,
	                                bool& modified) override;
	virtual
	void    AudioProcessorProcess (channel_t channel,
	                               float *buffer,
	                               int length, bool& modified) override;
//      coming from the GUI
	void	fax_setIOC              (const std::string &);
	void	fax_setMode             (const std::string &);
	void	fax_setPhase            (const std::string &);
	void	fax_setColor            (const std::string &);
	void	fax_setDeviation        (const std::string &);
	void	handle_resetButton	();
	void	handle_cheatButton	();
	void	set_correctionFactor	(int);
	void	handle_saveContinuous	();
	void	handle_saveSingle	();
	void	set_overflow		(int);
	void	regenerate		();
	
//
//	GUI setters
	void	show_faxState           (const std::string &);
	void	show_lineno             (int);
	void	show_savingLabel        (const std::string &);
	void	show_aptLabel		(int);

	enum Teint {
	   FAX_COLOR            = 1,
	   FAX_GRAY             = 2,
	   FAX_BLACKWHITE       = 3
	};

private:
//
//	we need some functions to get the data in from the SDRuno
	IUnoPluginController	*m_controller;
	SDRunoPlugin_faxUi      m_form;
	std::thread*		m_worker;
	RingBuffer<Complex>     inputBuffer;
	faxBandfilter	        passbandFilter;
	decimator_25	        theDecimator;
	LowPassFIR		audioFilter;
	RingBuffer<std::complex<float>> audioBuffer;
	faxShifter	        localMixer;
	std::vector<int>	faxLineBuffer;

	std::vector<float>	pixelStore;
	std::atomic<int>	overflow;
//
//
	std::atomic<bool>	running;
	faxParams	*getFaxParams	(const std::string &);
	int	        faxAudioRate;
	void	        WorkerFunction		();

	void		fax_setup	(const std::string &s);
	std::string	selected_IOC;

	void		processSample	(int);

	int	        checkFrequency	(std::vector<int> &, int, int, bool);
	int	        checkPhase	(std::vector<int> &, int, float);
	bool	        checkPhaseLine	(std::vector<int> &, int, float);
	int	        findPhaseLine	(std::vector<int> &, int, int, float);
	int	        shiftBuffer	(std::vector<int> &, int, int);
	void	        processBuffer	(std::vector<int> &, int, int);
	void		processLine	(std::vector<float> &,
	                                 std::vector<float> &, int, int);
	int	        demodulate	(std::complex<float> z);
//
//	These two talk to the FAX screen
	void	        clearScreen	();
	void		drawPicture	(int);

	int	        toRead;
	void	        addPixeltoImage	(float val, int, int);
	void	        saveImage_single	();
	void	        saveImage_auto		();

	std::vector<uint8_t>     rawData;
//
//	system wide parameters
	std::atomic<bool>	resetFlag;
	std::atomic<bool>	cheatFlag;
	int16_t         carrier;
	uint8_t	        faxState;	
	int 		deviation;
	int		bufferSize;
	std::atomic<bool> correcting;
	bool		setCorrection;
	bool	        saveContinuous;
	bool	        saveSingle;
	void	        doCorrection	();
	std::vector<int>	checkBuffer;
//
//	Mode specifics
	struct {
	   std::string	name;
	   int16_t      fax_IOC;
	   float	lpm;
	   int16_t	aptStartFreq;
	   int16_t	aptStopFreq;
	   bool		phaseInvers;
	   uint8_t	faxColor;
	   int32_t	samplesperLine; 
	   int16_t	nrColumns;
	   int		nrLines;
	   int		demodMode;
	} faxMode;
//
//	Fax instance specific parameters
	struct {
	   int		faxState;
	   int		bufferP;
	   int		checkP;
	   int		linesRecognized;
	   int		alarmCount;
	   int	        currentSampleIndex;
	   int16_t	lastRow; 
	   int	        stoppers;
	   int	        sampleOffset;
	   int		flipper;
	} theFax;
};
