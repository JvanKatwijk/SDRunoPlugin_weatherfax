#pragma once

#include <nana/gui.hpp>
#include <nana/gui/widgets/button.hpp>
#include <nana/gui/widgets/label.hpp>
#include <nana/gui/timer.hpp>
#include <iunoplugin.h>
#include <iostream>
#include <iomanip>
#include <sstream>

#include <iunoplugincontroller.h>
#include "SDRunoPlugin_faxForm.h"

// Forward reference
class	SDRunoPlugin_fax;

class	SDRunoPlugin_faxUi {
public:

	      SDRunoPlugin_faxUi	(SDRunoPlugin_fax& parent,
	                                 IUnoPluginController& controller);
	      ~SDRunoPlugin_faxUi	();

	void HandleEvent		(const UnoEvent& evt);
	void FormClosed			();

	void ShowUi		();
	int		LoadX		();
	int		LoadY		();
	std::string	loadDeviation	();
	std::string	load_phase	();
	std::string	load_ioc	();
	std::string	load_demodMode	();
	std::string	load_faxColor	();
	void		updateImage	();
	void		drawLine	(const std::vector<float> &, int);
//
//	going up
	void    fax_setIOC              (const std::string &);
        void    fax_setMode             (const std::string &);
        void    fax_setPhase            (const std::string &);
        void    fax_setColor            (const std::string &);
        void    fax_setDeviation        (const std::string &);
        void    handle_resetButton	();     
        void    handle_cheatButton	();
	void	set_correctionFactor	(int offset);
	void	regenerate		();
        void    handle_saveSingle	();
	void	set_overflow		(int);
        void    handle_saveContinuous	();
	nana::label	*getArea	();
	std::string	getDeviation	();
	std::string	get_phase	();
        std::string	get_ioc		();
        std::string	get_demodMode	();
        std::string	get_faxColor	();
	
//
//      coming down
        void    show_faxState           (const std::string &);
        void    show_lineno             (int);
        void    show_savingLabel        (const std::string &);
        void    show_aptLabel		(int);

private:
	
	SDRunoPlugin_fax & m_parent;
	std::thread m_thread;
	std::shared_ptr<SDRunoPlugin_faxForm> m_form;

	bool	m_started;

	std::mutex m_lock;

	IUnoPluginController &m_controller;
};
