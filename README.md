
--------------------------------------------------------------------------
SDRuno weatherFAX plugin
-------------------------------------------------------------------------

The "fax" plugin for SDRuno is a plugin for decoding weatherfax signals
transmitted on shortwave

![overview](/wfax-example.png?raw=true)

-----------------------------------------------------------------------------
  READ THIS FIRST installing the plugin
-----------------------------------------------------------------------------

Since the fax is a small band signal (less than 1 KHz in this plugin),
the samplerate used as input for the plugin is *62500* samples/second.

**On the main widget select samplerate 2000000, and decimation factor 32**.

![overview](/drm-main-widget.png?raw=true)

The plugin itself can be stored in the folder for community plugins

The plugin is - as other plugins - developed under MSVC. Its functioning
depends on lots of other "dll's" (Dynamic Load Libraries);

If a 0x000012f error is encountered on trying to load the plugin,
it means that dll's from the Visual C++ Redistributable(VS 2015) are
not found.


-----------------------------------------------------------------------
weatherfax
-----------------------------------------------------------------------

Even in this time of internet and satelites, shortwave ansmissions
are still used to transmit weathercharts.
The picture shows a (noise) signal, 4610 KHz,
with the weatherfax plugin.

The plugin assumes an inputrate of 2000000 / 32 (i.e. 62.5 KHz).
The frequencies on which weathercharts are transmitted are to be found in
e.g.

	https://www.weather.gov/media/marine/rfax.pdf

Most transmissions are preceded by a signal 400 Hz higher than the official
carrier frequency, a transmissions starts with a signal of 300 Hz (Phase
modulated) during a few seconds.

The implementation of the plugin is a reimplementation of the wfax
decoder of the swradio-8 implementation, that decoder was
reimplemented as well.

