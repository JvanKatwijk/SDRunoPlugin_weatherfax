#include <sstream>
#include <nana/gui.hpp>
#include <nana/gui/widgets/button.hpp>
#include <nana/gui/widgets/listbox.hpp>
#include <nana/gui/widgets/slider.hpp>
#include <nana/gui/widgets/label.hpp>
#include <nana/gui/timer.hpp>
#include <unoevent.h>

#include "SDRunoPlugin_fax.h"
#include "SDRunoPlugin_faxUi.h"
#include "SDRunoPlugin_faxForm.h"

// Ui constructor - load the Ui control into a thread
	SDRunoPlugin_faxUi::
	            SDRunoPlugin_faxUi (SDRunoPlugin_fax& parent,
	                                IUnoPluginController& controller):
	                                      m_parent (parent),
	                                      m_form (nullptr),
	                                      m_controller (controller) {
	m_thread = std::thread(&SDRunoPlugin_faxUi::ShowUi, this);
}

// Ui destructor (the nana::API::exit_all();) is required if using Nana UI library
	SDRunoPlugin_faxUi::~SDRunoPlugin_faxUi () {	
	nana::API::exit_all();
	m_thread.join ();	
}

// Show and execute the form
void	SDRunoPlugin_faxUi::ShowUi () {	
	m_lock.lock();
	m_form	= std::make_shared<SDRunoPlugin_faxForm>(*this, m_controller);
	m_lock.unlock();

	m_form -> Run ();
}

// Load X from the ini file (if exists)
// TODO: Change Template to plugin name
int	SDRunoPlugin_faxUi::LoadX () {
std::string tmp;
	m_controller.GetConfigurationKey ("fax.X", tmp);
	if (tmp.empty ()) {
	   return -1;
	}
	return stoi (tmp);
}

// Load Y from the ini file (if exists)
// TODO: Change Template to plugin name
int	SDRunoPlugin_faxUi::LoadY () {
std::string tmp;
	m_controller.GetConfigurationKey ("fax.Y", tmp);
	if (tmp.empty ()) {
	   return -1;
	}
	return stoi (tmp);
}
//
std::string	SDRunoPlugin_faxUi::loadDeviation () {
	std::string tmp = "";
	m_controller. GetConfigurationKey ("fax.Deviation", tmp);
	return tmp;
}

std::string	SDRunoPlugin_faxUi::getDeviation() {
	std::lock_guard<std::mutex> l(m_lock);
	if (m_form != nullptr)
		return m_form->getDeviation();
}
// Handle events from SDRuno
// TODO: code what to do when receiving relevant events
void	SDRunoPlugin_faxUi::HandleEvent (const UnoEvent& ev) {
	switch (ev.GetType()) {
	   case UnoEvent::StreamingStarted:
	      break;

	   case UnoEvent::StreamingStopped:
	      break;

	   case UnoEvent::SavingWorkspace:
	      break;

	   case UnoEvent::ClosingDown:
	      FormClosed ();
	      break;
	   default:
	      break;
	}
}

// Required to make sure the plugin is correctly unloaded when closed
void	SDRunoPlugin_faxUi::FormClosed () {
	m_controller.RequestUnload(&m_parent);
}

void	SDRunoPlugin_faxUi::fax_setIOC	(const std::string &s) {
	m_parent. fax_setIOC (s);
}

void	SDRunoPlugin_faxUi::fax_setMode (const std::string &s) {
	m_parent. fax_setMode (s);
}

void	SDRunoPlugin_faxUi::fax_setPhase (const std::string &s) {
	m_parent. fax_setPhase (s);
}

void	SDRunoPlugin_faxUi::fax_setColor (const std::string &s) {
	m_parent. fax_setColor (s);
}

void	SDRunoPlugin_faxUi::fax_setDeviation (const std::string &s) {
	m_parent. fax_setDeviation (s);
	m_controller.SetConfigurationKey ("fax.Deviation", s);
}

void	SDRunoPlugin_faxUi::handle_resetButton	() {
	m_parent. handle_resetButton ();
}
    
void	SDRunoPlugin_faxUi::handle_saveContinuous	() {
	m_parent. handle_saveContinuous ();
}

void	SDRunoPlugin_faxUi::handle_saveSingle	() {
	m_parent. handle_saveSingle ();
}

void	SDRunoPlugin_faxUi::handle_cheatButton	() {
	m_parent. handle_cheatButton ();
}

void    SDRunoPlugin_faxUi::set_correctionFactor      (int offset) {
        m_parent. set_correctionFactor (offset);
}

void    SDRunoPlugin_faxUi::regenerate        () {
        m_parent. regenerate ();
}

nana::label	*SDRunoPlugin_faxUi::getArea	() {
	return m_form -> getArea ();
}

//	and downwards
void	SDRunoPlugin_faxUi::show_faxState	(const std::string &s) {
	std::lock_guard<std::mutex> l (m_lock);
	if (m_form != nullptr)
	   m_form -> show_faxState (s);
}

void	SDRunoPlugin_faxUi::show_lineno	(int n) {
	std::lock_guard<std::mutex> l (m_lock);
        if (m_form != nullptr)
           m_form -> show_lineno (n);
}

void	SDRunoPlugin_faxUi::show_savingLabel	(const std::string &s) {
	std::lock_guard<std::mutex> l (m_lock);
	if (m_form != nullptr)
	   m_form -> show_saveLabel (s);
}

void	SDRunoPlugin_faxUi::show_aptLabel	(int n) {
	std::lock_guard<std::mutex> l (m_lock);
	if (m_form != nullptr)
	   m_form -> show_aptLabel (n);
}

