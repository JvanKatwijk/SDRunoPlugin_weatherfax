#include <sstream>
#ifdef _WIN32
#include <Windows.h>
#endif

#include "SDRunoPlugin_faxForm.h"
#include "SDRunoPlugin_faxUi.h"
#include "resource.h"
#include <io.h>
#include <shlobj.h>

float	xBuf[faxHeight * faxWidth];


//	Form constructor with handles to parent and uno controller -
//	launches form Setup
	SDRunoPlugin_faxForm::
	             SDRunoPlugin_faxForm (SDRunoPlugin_faxUi& parent,
	                                   IUnoPluginController& controller):
	nana::form(nana::API::make_center(formWidth, formHeight), nana::appearance(false, true, false, false, true, false, false)),
	m_parent(parent),
	m_controller(controller) {
	Setup();
}

// Form deconstructor
	SDRunoPlugin_faxForm::~SDRunoPlugin_faxForm () {
	faxContainer -> clear ();
	delete faxContainer;
}

// Start Form and start Nana UI processing
void	SDRunoPlugin_faxForm::Run	() {
	show();
	nana::exec();
}

// Create the initial plugin form
void	SDRunoPlugin_faxForm::Setup	() {
// This first section is all related to the background and border
	// it shouldn't need to be changed
	nana::paint::image img_border;
	nana::paint::image img_inner;
	HMODULE hModule = NULL;
	HRSRC rc_border = NULL;
	HRSRC rc_inner = NULL;
	HRSRC rc_close = NULL;
	HRSRC rc_close_over = NULL;
	HRSRC rc_min = NULL;
	HRSRC rc_min_over = NULL;
	HRSRC rc_bar = NULL;
	HRSRC rc_sett = NULL;
	HRSRC rc_sett_over = NULL;
	HBITMAP bm_border = NULL;
	HBITMAP bm_inner = NULL;
	HBITMAP bm_close = NULL;
	HBITMAP bm_close_over = NULL;
	HBITMAP bm_min = NULL;
	HBITMAP bm_min_over = NULL;
	HBITMAP bm_bar = NULL;
	HBITMAP bm_sett = NULL;
	HBITMAP bm_sett_over = NULL;
	BITMAPINFO bmInfo_border = { 0 };
	BITMAPINFO bmInfo_inner = { 0 };
	BITMAPINFO bmInfo_close = { 0 };
	BITMAPINFO bmInfo_close_over = { 0 };
	BITMAPINFO bmInfo_min = { 0 };
	BITMAPINFO bmInfo_min_over = { 0 };
	BITMAPINFO bmInfo_bar = { 0 };
	BITMAPINFO bmInfo_sett = { 0 };
	BITMAPINFO bmInfo_sett_over = { 0 };
	BITMAPFILEHEADER borderHeader = { 0 };
	BITMAPFILEHEADER innerHeader = { 0 };
	BITMAPFILEHEADER closeHeader = { 0 };
	BITMAPFILEHEADER closeoverHeader = { 0 };
	BITMAPFILEHEADER minHeader = { 0 };
	BITMAPFILEHEADER minoverHeader = { 0 };
	BITMAPFILEHEADER barHeader = { 0 };
	BITMAPFILEHEADER settHeader = { 0 };
	BITMAPFILEHEADER settoverHeader = { 0 };
	HDC hdc = NULL;
	BYTE* borderPixels = NULL;
	BYTE* innerPixels = NULL;
	BYTE* closePixels = NULL;
	BYTE* closeoverPixels = NULL;
	BYTE* minPixels = NULL;
	BYTE* minoverPixels = NULL;
	BYTE* barPixels = NULL;
	BYTE* barfocusedPixels = NULL;
	BYTE* settPixels = NULL;
	BYTE* settoverPixels = NULL;
	const unsigned int rawDataOffset = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFO);
	hModule = GetModuleHandle (L"SDRunoPlugin_fax");
	hdc = GetDC(NULL);
	rc_border = FindResource(hModule, MAKEINTRESOURCE(IDB_BG_BORDER), RT_BITMAP);
	rc_inner = FindResource(hModule, MAKEINTRESOURCE(IDB_BACKGROUND), RT_BITMAP);
	rc_close = FindResource(hModule, MAKEINTRESOURCE(IDB_CLOSE), RT_BITMAP);
	rc_close_over = FindResource(hModule, MAKEINTRESOURCE(IDB_CLOSE_DOWN), RT_BITMAP);
	rc_min = FindResource(hModule, MAKEINTRESOURCE(IDB_MIN), RT_BITMAP);
	rc_min_over = FindResource(hModule, MAKEINTRESOURCE(IDB_MIN_DOWN), RT_BITMAP);
	rc_bar = FindResource(hModule, MAKEINTRESOURCE(IDB_HEADER), RT_BITMAP);
	rc_sett = FindResource(hModule, MAKEINTRESOURCE(IDB_SETT), RT_BITMAP);
	rc_sett_over = FindResource(hModule, MAKEINTRESOURCE(IDB_SETT_DOWN), RT_BITMAP);
	bm_border = (HBITMAP)LoadImage(hModule, MAKEINTRESOURCE(IDB_BG_BORDER), IMAGE_BITMAP, 0, 0, LR_COPYFROMRESOURCE);
	bm_inner = (HBITMAP)LoadImage(hModule, MAKEINTRESOURCE(IDB_BACKGROUND), IMAGE_BITMAP, 0, 0, LR_COPYFROMRESOURCE);
	bm_close = (HBITMAP)LoadImage(hModule, MAKEINTRESOURCE(IDB_CLOSE), IMAGE_BITMAP, 0, 0, LR_COPYFROMRESOURCE);
	bm_close_over = (HBITMAP)LoadImage(hModule, MAKEINTRESOURCE(IDB_CLOSE_DOWN), IMAGE_BITMAP, 0, 0, LR_COPYFROMRESOURCE);
	bm_min = (HBITMAP)LoadImage(hModule, MAKEINTRESOURCE(IDB_MIN), IMAGE_BITMAP, 0, 0, LR_COPYFROMRESOURCE);
	bm_min_over = (HBITMAP)LoadImage(hModule, MAKEINTRESOURCE(IDB_MIN_DOWN), IMAGE_BITMAP, 0, 0, LR_COPYFROMRESOURCE);
	bm_bar = (HBITMAP)LoadImage(hModule, MAKEINTRESOURCE(IDB_HEADER), IMAGE_BITMAP, 0, 0, LR_COPYFROMRESOURCE);
	bm_sett = (HBITMAP)LoadImage(hModule, MAKEINTRESOURCE(IDB_SETT), IMAGE_BITMAP, 0, 0, LR_COPYFROMRESOURCE);
	bm_sett_over = (HBITMAP)LoadImage(hModule, MAKEINTRESOURCE(IDB_SETT_DOWN), IMAGE_BITMAP, 0, 0, LR_COPYFROMRESOURCE);
	bmInfo_border.bmiHeader.biSize = sizeof(bmInfo_border.bmiHeader);
	bmInfo_inner.bmiHeader.biSize = sizeof(bmInfo_inner.bmiHeader);
	bmInfo_close.bmiHeader.biSize = sizeof(bmInfo_close.bmiHeader);
	bmInfo_close_over.bmiHeader.biSize = sizeof(bmInfo_close_over.bmiHeader);
	bmInfo_min.bmiHeader.biSize = sizeof(bmInfo_min.bmiHeader);
	bmInfo_min_over.bmiHeader.biSize = sizeof(bmInfo_min_over.bmiHeader);
	bmInfo_bar.bmiHeader.biSize = sizeof(bmInfo_bar.bmiHeader);
	bmInfo_sett.bmiHeader.biSize = sizeof(bmInfo_sett.bmiHeader);
	bmInfo_sett_over.bmiHeader.biSize = sizeof(bmInfo_sett_over.bmiHeader);
	GetDIBits(hdc, bm_border, 0, 0, NULL, &bmInfo_border, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_inner, 0, 0, NULL, &bmInfo_inner, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_close, 0, 0, NULL, &bmInfo_close, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_close_over, 0, 0, NULL, &bmInfo_close_over, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_min, 0, 0, NULL, &bmInfo_min, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_min_over, 0, 0, NULL, &bmInfo_min_over, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_bar, 0, 0, NULL, &bmInfo_bar, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_sett, 0, 0, NULL, &bmInfo_sett, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_sett_over, 0, 0, NULL, &bmInfo_sett_over, DIB_RGB_COLORS);
	bmInfo_border.bmiHeader.biCompression = BI_RGB;
	bmInfo_inner.bmiHeader.biCompression = BI_RGB;
	bmInfo_close.bmiHeader.biCompression = BI_RGB;
	bmInfo_close_over.bmiHeader.biCompression = BI_RGB;
	bmInfo_min.bmiHeader.biCompression = BI_RGB;
	bmInfo_min_over.bmiHeader.biCompression = BI_RGB;
	bmInfo_bar.bmiHeader.biCompression = BI_RGB;
	bmInfo_sett.bmiHeader.biCompression = BI_RGB;
	bmInfo_sett_over.bmiHeader.biCompression = BI_RGB;
	borderHeader.bfOffBits = rawDataOffset;
	borderHeader.bfSize = bmInfo_border.bmiHeader.biSizeImage;
	borderHeader.bfType = 0x4D42;
	innerHeader.bfOffBits = rawDataOffset;
	innerHeader.bfSize = bmInfo_inner.bmiHeader.biSizeImage;
	innerHeader.bfType = 0x4D42;
	closeHeader.bfOffBits = rawDataOffset;
	closeHeader.bfSize = bmInfo_close.bmiHeader.biSizeImage;
	closeHeader.bfType = 0x4D42;
	closeoverHeader.bfOffBits = rawDataOffset;
	closeoverHeader.bfSize = bmInfo_close_over.bmiHeader.biSizeImage;
	closeoverHeader.bfType = 0x4D42;
	minHeader.bfOffBits = rawDataOffset;
	minHeader.bfSize = bmInfo_min.bmiHeader.biSizeImage;
	minHeader.bfType = 0x4D42;
	minoverHeader.bfOffBits = rawDataOffset;
	minoverHeader.bfSize = bmInfo_min_over.bmiHeader.biSizeImage;
	minoverHeader.bfType = 0x4D42;
	barHeader.bfOffBits = rawDataOffset;
	barHeader.bfSize = bmInfo_bar.bmiHeader.biSizeImage;
	barHeader.bfType = 0x4D42;
	settHeader.bfOffBits = rawDataOffset;
	settHeader.bfSize = bmInfo_sett.bmiHeader.biSizeImage;
	settHeader.bfType = 0x4D42;
	settoverHeader.bfOffBits = rawDataOffset;
	settoverHeader.bfSize = bmInfo_sett_over.bmiHeader.biSizeImage;
	settoverHeader.bfType = 0x4D42;
	borderPixels = new BYTE[bmInfo_border.bmiHeader.biSizeImage + rawDataOffset];
	innerPixels = new BYTE[bmInfo_inner.bmiHeader.biSizeImage + rawDataOffset];
	closePixels = new BYTE[bmInfo_close.bmiHeader.biSizeImage + rawDataOffset];
	closeoverPixels = new BYTE[bmInfo_close_over.bmiHeader.biSizeImage + rawDataOffset];
	minPixels = new BYTE[bmInfo_min.bmiHeader.biSizeImage + rawDataOffset];
	minoverPixels = new BYTE[bmInfo_min_over.bmiHeader.biSizeImage + rawDataOffset];
	barPixels = new BYTE[bmInfo_bar.bmiHeader.biSizeImage + rawDataOffset];
	settPixels = new BYTE[bmInfo_sett.bmiHeader.biSizeImage + rawDataOffset];
	settoverPixels = new BYTE[bmInfo_sett_over.bmiHeader.biSizeImage + rawDataOffset];
	*(BITMAPFILEHEADER*)borderPixels = borderHeader;
	*(BITMAPINFO*)(borderPixels + sizeof(BITMAPFILEHEADER)) = bmInfo_border;
	*(BITMAPFILEHEADER*)innerPixels = innerHeader;
	*(BITMAPINFO*)(innerPixels + sizeof(BITMAPFILEHEADER)) = bmInfo_inner;
	*(BITMAPFILEHEADER*)closePixels = closeHeader;
	*(BITMAPINFO*)(closePixels + sizeof(BITMAPFILEHEADER)) = bmInfo_close;
	*(BITMAPFILEHEADER*)closeoverPixels = closeoverHeader;
	*(BITMAPINFO*)(closeoverPixels + sizeof(BITMAPFILEHEADER)) = bmInfo_close_over;
	*(BITMAPFILEHEADER*)minPixels = minHeader;
	*(BITMAPINFO*)(minPixels + sizeof(BITMAPFILEHEADER)) = bmInfo_min;
	*(BITMAPFILEHEADER*)minoverPixels = minoverHeader;
	*(BITMAPINFO*)(minoverPixels + sizeof(BITMAPFILEHEADER)) = bmInfo_min_over;
	*(BITMAPFILEHEADER*)barPixels = barHeader;
	*(BITMAPINFO*)(barPixels + sizeof(BITMAPFILEHEADER)) = bmInfo_bar;
	*(BITMAPFILEHEADER*)settPixels = settHeader;
	*(BITMAPINFO*)(settPixels + sizeof(BITMAPFILEHEADER)) = bmInfo_sett;
	*(BITMAPFILEHEADER*)settoverPixels = settoverHeader;
	*(BITMAPINFO*)(settoverPixels + sizeof(BITMAPFILEHEADER)) = bmInfo_sett_over;
	GetDIBits(hdc, bm_border, 0, bmInfo_border.bmiHeader.biHeight, (LPVOID)(borderPixels + rawDataOffset), &bmInfo_border, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_inner, 0, bmInfo_inner.bmiHeader.biHeight, (LPVOID)(innerPixels + rawDataOffset), &bmInfo_inner, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_close, 0, bmInfo_close.bmiHeader.biHeight, (LPVOID)(closePixels + rawDataOffset), &bmInfo_close, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_close_over, 0, bmInfo_close_over.bmiHeader.biHeight, (LPVOID)(closeoverPixels + rawDataOffset), &bmInfo_close_over, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_min, 0, bmInfo_min.bmiHeader.biHeight, (LPVOID)(minPixels + rawDataOffset), &bmInfo_min, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_min_over, 0, bmInfo_min_over.bmiHeader.biHeight, (LPVOID)(minoverPixels + rawDataOffset), &bmInfo_min_over, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_bar, 0, bmInfo_bar.bmiHeader.biHeight, (LPVOID)(barPixels + rawDataOffset), &bmInfo_bar, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_sett, 0, bmInfo_sett.bmiHeader.biHeight, (LPVOID)(settPixels + rawDataOffset), &bmInfo_sett, DIB_RGB_COLORS);
	GetDIBits(hdc, bm_sett_over, 0, bmInfo_sett_over.bmiHeader.biHeight, (LPVOID)(settoverPixels + rawDataOffset), &bmInfo_sett_over, DIB_RGB_COLORS);
	img_border.open(borderPixels, bmInfo_border.bmiHeader.biSizeImage);
	img_inner.open(innerPixels, bmInfo_inner.bmiHeader.biSizeImage);
	img_close_normal.open(closePixels, bmInfo_close.bmiHeader.biSizeImage);
	img_close_down.open(closeoverPixels, bmInfo_close_over.bmiHeader.biSizeImage);
	img_min_normal.open(minPixels, bmInfo_min.bmiHeader.biSizeImage);
	img_min_down.open(minoverPixels, bmInfo_min_over.bmiHeader.biSizeImage);
	img_header.open(barPixels, bmInfo_bar.bmiHeader.biSizeImage);
	img_sett_normal.open(settPixels, bmInfo_sett.bmiHeader.biSizeImage);
	img_sett_down.open(settoverPixels, bmInfo_sett_over.bmiHeader.biSizeImage);
	ReleaseDC(NULL, hdc);
	bg_border.load(img_border, nana::rectangle(0, 0, 590, 340));
	bg_border.stretchable(0, 0, 0, 0);
	bg_border.transparent(true);
	bg_inner.load(img_inner, nana::rectangle(0, 0, 582, 299));
	bg_inner.stretchable(sideBorderWidth, 0, sideBorderWidth, bottomBarHeight);
	bg_inner.transparent(false);

	// TODO: Form code starts here

	// Load X and Y location for the form from the ini file (if exists)
	int posX = m_parent.LoadX();
	int posY = m_parent.LoadY();
	move(posX, posY);

	// This code sets the plugin size, title and what to do when the X is pressed
	size(nana::size(formWidth, formHeight));
	caption ("SDRuno weatherfax-2 plugin");
	events().destroy([&] { m_parent.FormClosed(); });

	//Initialize header bar
	header_bar.size(nana::size(122, 20));
	header_bar.load(img_header, nana::rectangle(0, 0, 122, 20));
	header_bar.stretchable(0, 0, 0, 0);
	header_bar.move(nana::point((formWidth / 2) - 61, 5));
	header_bar.transparent(true);

	//Initial header text 
	title_bar_label.size(nana::size(65, 12));
	title_bar_label.move(nana::point((formWidth / 2) - 5, 9));
	title_bar_label.format(true);
	title_bar_label.caption("< bold size = 6 color = 0x000000 font = \"Verdana\">weatherfax-2</>");
	title_bar_label.text_align(nana::align::center, nana::align_v::center);
	title_bar_label.fgcolor(nana::color_rgb(0x000000));
	title_bar_label.transparent(true);

	//Iniitialize drag_label
	form_drag_label.move(nana::point(0, 0));
	form_drag_label.transparent(true);

	//Initialize dragger and set target to form, and trigger to drag_label 
	form_dragger.target(*this);
	form_dragger.trigger(form_drag_label);

	//Initialise the "Minimize button"
	min_button.load(img_min_normal, nana::rectangle(0, 0, 20, 15));
	min_button.bgcolor(nana::color_rgb(0x000000));
	min_button.move(nana::point(formWidth - 51, 9));
	min_button.transparent(true);
	min_button.events().mouse_down([&] { min_button.load(img_min_down, nana::rectangle(0, 0, 20, 15)); });
	min_button.events().mouse_up([&] { min_button.load(img_min_normal, nana::rectangle(0, 0, 20, 15)); nana::API::zoom_window(this->handle(), false); });
	min_button.events().mouse_leave([&] { min_button.load(img_min_normal, nana::rectangle(0, 0, 20, 15)); });

	//Initialise the "Close button"
	close_button.load(img_close_normal, nana::rectangle(0, 0, 20, 15));
	close_button.bgcolor(nana::color_rgb(0x000000));
	close_button.move(nana::point(formWidth - 26, 9));
	close_button.transparent(true);
	close_button.events().mouse_down([&] { close_button.load(img_close_down, nana::rectangle(0, 0, 20, 15)); });
	close_button.events().mouse_up([&] { close_button.load(img_close_normal, nana::rectangle(0, 0, 20, 15)); close(); });
	close_button.events().mouse_leave([&] { close_button.load(img_close_normal, nana::rectangle(0, 0, 20, 15)); });

	//Uncomment the following block of code to Initialise the "Setting button"
	sett_button.load(img_sett_normal, nana::rectangle(0, 0, 40, 15));
	sett_button.bgcolor(nana::color_rgb(0x000000));
	sett_button.move(nana::point(10, 9));
	sett_button.events().mouse_down([&] { sett_button.load(img_sett_down, nana::rectangle(0, 0, 40, 15)); });
	sett_button.events().mouse_up([&] { sett_button.load(img_sett_normal, nana::rectangle(0, 0, 40, 15)); SettingsButton_Click(); });
	sett_button.events().mouse_leave([&] { sett_button.load(img_sett_normal, nana::rectangle(0, 0, 40, 15)); });
	sett_button.tooltip("Show settings window");
	sett_button.transparent(true);

	// TODO: Extra Form code goes here

//	we draw the map on an label with a size of faxWidth x faxHeight
	faxContainer = new drawing (imageLabel);
	faxContainer -> draw ([&](paint::graphics& graph) {
	        for (int i = 0; i < faxHeight; i ++) {
	           for (int j = 0; j < faxWidth; j ++) {
	              float res = xBuf [i * faxWidth + j];
	              graph.set_pixel (j, i, res >= 128 ?
	                                          nana::colors::white :
	                                          nana::colors::black);
	           }
	        }
		});
	std::string iocInd	= m_parent. load_ioc ();
	faxIOC	.push_back ("Wefax576");
	faxIOC	.push_back ("Wefax288");
	if (iocInd == "Wefax288")
	   faxIOC. option (1);
	else
	   faxIOC .option (0);
	faxIOC	.events (). selected ([&] (const nana::arg_combox &ar_cbx)
                            {fax_setIOC (ar_cbx. widget. caption ());});
        faxIOC. tooltip ("IOC");

	std::string demodMode	= m_parent. load_demodMode ();
	faxMode	.push_back ("FM");
	faxMode	.push_back ("AM");
	if (demodMode == "AM")
	   faxMode. option (1);
	else
	   faxMode. option (0);
	faxMode	.events (). selected ([&] (const nana::arg_combox &ar_cbx)
                            {fax_setMode (ar_cbx. widget. caption ());});
        faxMode. tooltip ("Mode AM or FM");

	std::string phaseInd = m_parent. load_phase ();
	faxPhase. push_back ("phase");
	faxPhase. push_back ("inverse");
	if (phaseInd == "inverse")
		faxPhase.option(1);
	else
		faxPhase.option(0);
	faxPhase .events (). selected ([&] (const nana::arg_combox &ar_cbx)
                            {fax_setPhase (ar_cbx. widget. caption ());});
        faxMode. tooltip ("Use normal phasing or inverse phase for decoding");

	std::string colorInd	= m_parent. load_faxColor ();
        faxColor .push_back ("BW");
	faxColor .push_back ("GRAY");
	faxColor .push_back ("COLOR");
	if (colorInd == "GRAY")
	   faxColor. option (1);
	else
	if (colorInd == "COLOR")
	   faxColor. option (2);
	else
	   faxColor .option (0);
	faxColor .events (). selected ([&] (const nana::arg_combox &ar_cbx)
                            {fax_setColor (ar_cbx. widget. caption ());});
        faxColor. tooltip ("Select a color mode");
//
//	load a predefined value for the deviation, if any
	std::string deviationInd = m_parent. loadDeviation ();
	deviation. push_back ("1900-400");
	deviation. push_back ("1950-450");
	if (deviationInd == "1900-400")
	   deviation. option (0);
	if (deviationInd == "1950-450")
	   deviation.option (1);
	else
	   deviation.option(0);

	deviation .events (). selected ([&] (const nana::arg_combox &ar_cbx)
                              {fax_setDeviation (ar_cbx. widget. caption ());});
        deviation. tooltip ("Europe 1900-400, US 1950-450");

	overflow. range (0, 300, 20);
	overflow. value (std::to_string (50));
	overflow.
           events (). text_changed ([&](const nana::arg_spinbox &s) {
                                      set_overflow (overflow. to_int ());});

	saveContinuous. caption ("save Cont");
        saveContinuous. events(). click ([&]() {handle_saveContinuous ();});

	saveSingle. caption ("save Sngl");
        saveSingle. events(). click ([&]() {handle_saveSingle ();});

	resetButton. caption ("reset");
        resetButton. events(). click ([&]() {handle_resetButton ();});

	cheatButton. caption ("cheat");
	cheatButton. events (). click ([&]() {handle_cheatButton ();});

	faxState	.transparent (true);
	faxState	.fgcolor (nana::colors::white);

	copyRightLabel	.transparent (true);
	copyRightLabel	.fgcolor (nana::colors::white);
	copyRightLabel	.caption ("\xa9");
	copyRightLabel	.tooltip ("created by Jan van Katwijk, all rights reserved");

	aptLabel	.transparent(true);
	aptLabel	.fgcolor(nana::colors::white);

	lineno		.transparent (true);
	lineno		.fgcolor (nana::colors::white);

	sampleCorrection. range (-20, 20, 1);
	sampleCorrection. value (std::to_string (0));
	sampleCorrection.
	   events (). text_changed ([&](const nana::arg_spinbox &s) {
                                      set_correctionFactor (sampleCorrection. to_int ());});
	sampleCorrection. tooltip ("set nr samples to correct line length");
	sampleCorrection. bgcolor (nana::colors::black);
	sampleCorrection. fgcolor (nana::colors::white);

	correctButton. caption ("correct");
	correctButton. events (). click ([&](){regenerate ();});
        correctButton. fgcolor(nana::colors::white);
        correctButton. bgcolor(nana::colors::black);
	correctButton. tooltip ("use to correct the picture after picture completion");
	delete[] borderPixels;
	delete[] innerPixels;
	delete[] closePixels;
	delete[] closeoverPixels;
	delete[] minPixels;
	delete[] minoverPixels;
	delete[] barPixels;
	delete[] barfocusedPixels;
	delete[] settPixels;
	delete[] settoverPixels;
}

void	SDRunoPlugin_faxForm::SettingsButton_Click () {
	// TODO: Insert code here to show settings panel
}
//

void	SDRunoPlugin_faxForm::fax_setIOC	(const std::string &s) {
	m_parent. fax_setIOC (s);
}

void	SDRunoPlugin_faxForm::fax_setMode	(const std::string &s) {
	m_parent. fax_setMode (s);
}

void	SDRunoPlugin_faxForm::fax_setPhase	(const std::string &s) {
	m_parent.fax_setPhase (s);
}

void	SDRunoPlugin_faxForm::fax_setColor	(const std::string &s) {
	m_parent. fax_setColor (s);
}

void	SDRunoPlugin_faxForm::fax_setDeviation	(const std::string &s) {
	m_parent. fax_setDeviation (s);
}

void	SDRunoPlugin_faxForm::handle_saveContinuous  () {
	m_parent. handle_saveContinuous ();
}

void	SDRunoPlugin_faxForm::handle_saveSingle  () {
	m_parent. handle_saveSingle ();
}

void	SDRunoPlugin_faxForm::set_overflow (int n) {
	m_parent. set_overflow (n);
}

void	SDRunoPlugin_faxForm::handle_resetButton	() {
	m_parent. handle_resetButton ();
}

//      coming down

void	SDRunoPlugin_faxForm::show_faxState	(const std::string &s) {
	faxState. caption (s);
}

void	SDRunoPlugin_faxForm::show_lineno	(int n) {
	lineno. caption (std::to_string (n));
}

void	SDRunoPlugin_faxForm::show_saveLabel	(const std::string &s) {
	saveContinuous. caption (s);
}

void	SDRunoPlugin_faxForm::show_aptLabel	(int n) {
	aptLabel. caption (std::to_string (n));
}

nana::label *SDRunoPlugin_faxForm::getArea () {
	return &imageLabel;
}


void	SDRunoPlugin_faxForm::handle_cheatButton	() {
	m_parent. handle_cheatButton ();
}

void	SDRunoPlugin_faxForm::set_correctionFactor	(int offset) {
	m_parent. set_correctionFactor (offset);
}

void	SDRunoPlugin_faxForm::regenerate 	() {
	m_parent. regenerate ();
}

std::string	SDRunoPlugin_faxForm::getDeviation	() {
	return deviation. text (deviation. option ());
}

std::string	SDRunoPlugin_faxForm::get_phase	() {
	return faxPhase. text (faxPhase. option ());
}

std::string	SDRunoPlugin_faxForm::get_ioc	() {
	return faxIOC. text (faxIOC. option ());
}

std::string	SDRunoPlugin_faxForm::get_demodMode	() {
	return faxMode. text (faxMode. option ());
}

std::string	SDRunoPlugin_faxForm::get_faxColor	() {
	return faxColor. text (faxColor. option ());
}

void	SDRunoPlugin_faxForm::drawLine	(const std::vector<float> &v, int l) {
	for (int i = 0; i < faxWidth; i ++)
	   xBuf [l * faxWidth + i] = v [i];
}

void	SDRunoPlugin_faxForm::updateImage	() {
	faxContainer -> update ();
}

