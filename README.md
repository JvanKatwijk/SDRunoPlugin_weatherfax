
--------------------------------------------------------------------------
SDRuno weatherFAX plugin
-------------------------------------------------------------------------

The "fax" plugin for SDRuno is a plugin for decoding weatherfax signals
transmitted on shortwave

![overview](/wfax-example.png?raw=true)

-----------------------------------------------------------------------------
  READ THIS FIRST installing the plugin
-----------------------------------------------------------------------------

The weatherfax signal has a width of less than 1 KHz, since the minimum width
of the samplerate from SDR devices is 2 MHz, there is some decimation
required.

In this version, the plugin uses the SP1 stream, a data stream of 192 KHz,
provided for by the SDRuno platform.

It seems wide though to use the zooming facility of the main spectrum widget
to make the signal visible, a "shadow" band will show in which the signal
of interest can be found.

The SP exit is selected by the plugin!

![overview](/drm-main-widget.png?raw=true)

The plugin itself can be stored in the folder for community plugins

The plugin is - as other plugins - developed under MSVC. Its functioning
depends on lots of other "dll's" (Dynamic Load Libraries);

If a 0x000012f error is encountered on trying to load the plugin,
it means that dll's from the Visual C++ Redistributable(VS 2015) are
not found.

Installing seems to be quite simple

How do I fix the api-ms-win-crt-runtime-l1-1-0. dll missing error?

    Install the software via Windows Update.
    Download Visual C++ Redistributable for Visual Studio 2015 from Microsoft directly.
    Install or Repair the Visual C++ Redistributable for Visual Studio 2015 on your computer.


-----------------------------------------------------------------------
weatherfax
-----------------------------------------------------------------------

Even in this time of internet and satelites, shortwave ansmissions
are still used to transmit weathercharts.
The picture shows a (noise) signal, 4610 KHz,
with the weatherfax plugin.

The frequencies on which weathercharts are transmitted are to be found in
e.g.

	https://www.weather.gov/media/marine/rfax.pdf

Most transmissions are preceded by a signal 400 Hz higher than the official
carrier frequency, a transmissions starts with a signal of 300 Hz (Phase
modulated) during a few seconds.

The implementation of the plugin is a reimplementation of the wfax
decoder of the swradio-8 implementation, that decoder was
reimplemented as well.

