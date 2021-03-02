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
#include	"utilities.h"
#include	"fax-demodulator.h"  

#define         DECIMATOR       5
#define         INRATE          (2000000 / 32)
#define         INTERM_RATE     (INRATE / DECIMATOR)
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
#define	CHEATING		        0000

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
        void	handle_saveButton       ();
        void	handle_cheatButton	();
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
	std::mutex		m_lock;
	SDRunoPlugin_faxUi      m_form;
	std::mutex	        locker;
	IUnoPluginController	*m_controller;
	RingBuffer<Complex>     inputBuffer;
	faxShifter	        theMixer;
	faxBandfilter	        passbandFilter;
	decimator_25	        theDecimator;
	faxShifter	        localMixer;
	RingBuffer<float>	faxAudioBuffer;
	button			faxForm;
	drawing			*faxContainer;
	std::vector<float>	pixelStore;

	std::vector<std::complex<float>> faxToneBuffer;
	std::vector<std::complex<float>> convBuffer;
	int	     convIndex;
	int16_t	     mapTable_int   [WORKING_RATE / 100];
	float	     mapTable_float [WORKING_RATE / 100];
//
//
	std::atomic<bool>	running;
	faxParams	*getFaxParams	(const std::string &);
	void	    setup_faxDecoder(std::string IOC_name);
	int	    centerFrequency;
	int	    VFOFRequency;
	int	    selectedFrequency;
	int	    faxTonePhase;
	int	    Raw_Rate;
	int	    faxAudioRate;
	bool	    faxError;
	void	    WorkerFunction		();
	std::thread*	    m_worker;

	upFilter	*audioFilter;
	void	        process	        (std::complex<float>);
	int	        resample	(std::complex<float>,
	                                std::complex<float> *);
	void	        processSample	(std::complex<float>);

	int	        checkFrequency	(std::vector<int>, int, int);
	int	        checkPhase	(int, float);
	bool	        checkPhaseLine	(int, float);
	int	        findPhaseLine	(int, int, float);
	int	        shiftBuffer	(std::vector<int>, int, int);
	void	        processBuffer	(int);
	int	        toRead;
	void	        addPixeltoImage	(float val, int, int);
	void	        saveImage       ();
	void	        clearScreen	();

	faxDemodulator  *myDemodulator;
	LowPassFIR      *faxLowPass;
	faxAverage*	faxAverager;
	std::vector<uint8_t>     rawData;
	
	uint8_t	        faxState;
	int16_t	        fax_IOC;
	float	        lpm;
	float	        f_lpm;
	int16_t	        deviation;
	int16_t	        apt_upCrossings;
	int16_t	        aptStartFreq;
	int16_t	        aptStopFreq;
	bool            phaseInvers;
	uint8_t         faxColor;
	int16_t         carrier;
	uint8_t         faxMode;
	int32_t         samplesperLine; 
	int16_t         numberofColumns;
	int             nrLines;
	std::vector<int>	faxLineBuffer;
	std::vector<int>	checkBuffer;
	int             bufferP;
	int	        checkP;
	int	        bufferSize;
	int             linesRecognized;
	bool            savingRequested;
	int	        alarmCount;

	int	        currentSampleIndex;
	int16_t         lastRow; 
	int32_t         pixelValue; 
	float           pixelSamples;
	int	        stoppers;
};
