#pragma once

#include <nana/gui.hpp>
#include <nana/gui/widgets/button.hpp>
#include <nana/gui/widgets/listbox.hpp>
#include <nana/gui/widgets/slider.hpp>
#include <nana/gui/widgets/label.hpp>
#include <nana/gui/widgets/combox.hpp>
#include <nana/gui/widgets/spinbox.hpp>
#include <nana/gui/widgets/scroll.hpp>

#include <nana/gui/timer.hpp>
#include <nana/gui/widgets/picture.hpp>
#include <nana/gui/filebox.hpp>
#include <nana/gui/dragger.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>

#include <iunoplugincontroller.h>

using namespace nana;

// Shouldn't need to change these
#define topBarHeight (27)
#define bottomBarHeight (8)
#define sideBorderWidth (8)

// TODO: Change these numbers to the height and width of your form
#define formWidth (960)
#define formHeight (780)

#define	faxWidth	(formWidth - 60)
#define	faxHeight	(formHeight - 100)


class SDRunoPlugin_faxUi;

class SDRunoPlugin_faxForm : public nana::form {

public:

		SDRunoPlugin_faxForm	(SDRunoPlugin_faxUi& parent,
	                                 IUnoPluginController& controller);		
		~SDRunoPlugin_faxForm	();
	void	Run			();
//
//	going up
	void	fax_setIOC		(const std::string &);
	void	fax_setMode		(const std::string &);
	void	fax_setPhase		(const std::string &);
	void	fax_setColor		(const std::string &);
	void	fax_setDeviation	(const std::string &);
	void	handle_resetButton	();
	void	handle_cheatButton	();
	void	set_correctionFactor    (int);
	void	handle_saveContinuous	();
	void	handle_saveSingle	();
	void	set_overflow		(int);
	void	regenerate       	();
	std::string getDeviation	();
	std::string get_phase		();
	std::string get_ioc		();
	std::string get_demodMode	();
	std::string get_faxColor	();
	nana::label *getArea		();
	void	updateImage		();
	void	drawLine		(const std::vector<float> &, int);
//
//	coming down
	void	show_faxState		(const std::string &);
	void	show_lineno		(int);
	void	show_saveLabel		(const std::string &);
	void	show_aptLabel		(int);

private:
	void	Setup			();

// The following is to set up the panel graphic to look like a standard SDRuno panel
	nana::picture bg_border{ *this, nana::rectangle(0, 0, formWidth, formHeight) };
	nana::picture bg_inner{ bg_border, nana::rectangle(sideBorderWidth, topBarHeight, formWidth - (2 * sideBorderWidth), formHeight - topBarHeight - bottomBarHeight) };
	nana::picture header_bar{ *this, true };
	nana::label title_bar_label{ *this, true };
	nana::dragger form_dragger;
	nana::label form_drag_label{ *this, nana::rectangle(0, 0, formWidth, formHeight) };
	nana::paint::image img_min_normal;
	nana::paint::image img_min_down;
	nana::paint::image img_close_normal;
	nana::paint::image img_close_down;
	nana::paint::image img_header;
	nana::picture close_button{ *this, nana::rectangle(0, 0, 20, 15) };
	nana::picture min_button{ *this, nana::rectangle(0, 0, 20, 15) };

	// Uncomment the following 4 lines if you want a SETT button and panel
	nana::paint::image img_sett_normal;
	nana::paint::image img_sett_down;
	nana::picture sett_button{ *this, nana::rectangle(0, 0, 40, 15) };
	void SettingsButton_Click();

	// TODO: Now add your UI controls here

	nana::combox	faxIOC	{*this,
	                            nana::rectangle (30, 30, 100, 20)};
	nana::combox	faxMode{ *this,
	                            nana::rectangle (140, 30, 100, 20)};
	nana::combox	faxPhase {*this,
	                            nana::rectangle (250, 30, 100, 20)};
	nana::combox	faxColor {*this,
	                            nana::rectangle (360, 30, 100, 20)};
	nana::combox	deviation {*this,
	                            nana::rectangle (470, 30, 100, 20)};
	nana::label	theName  {*this,
	                            nana::rectangle (580, 30, 100, 20)};
	nana::label	copyRightLabel {*this, 
	                            nana::rectangle (700, 30, 30, 20)};
	nana::label	faxState {*this,
	                            nana::rectangle (740, 30, 140, 20)};
	nana::button	saveContinuous  {*this,
	                            nana::rectangle (30, 60, 80, 20)};
	nana::button	resetButton {*this,
	                            nana::rectangle (120, 60, 80, 20)};
	nana::label	aptLabel {*this,
	                            nana::rectangle (210, 60, 80, 20)};
	nana::label	lineno {*this,
	                            nana::rectangle (300, 60, 80, 20)};
	nana::button	cheatButton {*this,
	                            nana::rectangle (390, 60, 80, 20)};
	nana::button	correctButton {*this, 
	                            nana::rectangle (480, 60, 80, 20)};
	nana::spinbox	sampleCorrection { *this,
	                            nana::rectangle (570, 60, 80, 20) };
	nana::button	saveSingle {*this,
	                            nana::rectangle (660, 60, 80, 20) };
	nana::spinbox	overflow {*this,
	                            nana::rectangle (750, 60, 80, 20) };
	nana::label	imageLabel {*this, nana::rectangle (30, 90,
	                                                    faxWidth,
	                                                    faxHeight)};
	drawing		*faxContainer;
	SDRunoPlugin_faxUi &m_parent;
	IUnoPluginController &m_controller;
};
